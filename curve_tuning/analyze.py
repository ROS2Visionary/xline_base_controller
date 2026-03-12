#!/usr/bin/env python3
"""
curve_tracking_latest 综合诊断脚本

用法（从项目根目录运行）:
  python3 curve_tuning/analyze.py                  # 分析最新批次（自动深析最差曲线）
  python3 curve_tuning/analyze.py --curve 01       # 指定曲线深层分析
  python3 curve_tuning/analyze.py --compare        # 输出全历史批次对比
  python3 curve_tuning/analyze.py --all-deep       # 全曲线三层深度分析
"""

import sys, os, math, argparse
import pandas as pd
import numpy as np
from pathlib import Path

# ─────────────────── 配置 ───────────────────
DATA_DIR  = Path(__file__).parent.parent / "curve_tracking_latest"
HIST_FILE = Path(__file__).parent / "curve_optimization_history.md"

# 历史批次基准值（每次测试后手动填入）
# 格式: {"C<批次号>_<batch_id>": {曲线槽: {lt3, lt5, p90, mean_signed}}}
BASELINES = {
    # 示例（首次测试后填入）：
    # "C00_<batch_id>": {
    #     0: {"lt3": 0.0, "lt5": 0.0, "p90": 0.0, "mean_signed": 0.0},
    # },
}

# ─────────────────── 工具函数 ───────────────────
def hr(char="─", n=78): print(char * n)
def section(title): hr("═"); print(f"  {title}"); hr("═")
def subsec(title):  hr("─"); print(f"  {title}"); hr("─")

def pass_mark(val, threshold, lower_is_better=True):
    if lower_is_better:
        return "✓" if val < threshold else "✗"
    else:
        return "✓" if val >= threshold else "✗"

def zeta_curve(k1, k2, v):
    """
    曲线 LQR 阻尼比（近似，以前馈为主时仅供参考）
    ζ = K2 / (2·√(v·K1))
    """
    if v <= 0 or k1 <= 0: return float('nan')
    return k2 / (2.0 * math.sqrt(v * k1))


# ─────────────────── 层1：批次概览 ───────────────────
def layer1_batch(bm: pd.DataFrame) -> dict:
    opt_id    = bm["optimization_id"].iloc[0] if "optimization_id" in bm.columns else "?"
    batch_id  = bm["batch_id"].iloc[0]
    parent_id = bm["parent_batch_id"].iloc[0] if "parent_batch_id" in bm.columns else "?"

    section(f"层1 · 批次概览  (batch={batch_id})")
    print(f"  优化ID      : {opt_id}")
    print(f"  父批次      : {parent_id}")

    # 找父批次基准
    prev_label = None
    for label in reversed(list(BASELINES.keys())):
        if str(parent_id) in label:
            prev_label = label
            break
    prev_data = BASELINES.get(prev_label, {})

    # 表头
    print()
    hdr  = f"  {'曲线':>6}  {'路径类型':>8}  {'总长(m)':>8}  "
    hdr += f"{'p90(mm)':>8}  {'<3mm%':>7}  {'<5mm%':>7}  "
    hdr += f"{'偏向(mm)':>10}  {'通过':>5}"
    if prev_label:
        hdr += f"  {'vs父批次':>10}"
    print(hdr)
    hr()

    results = {}
    for _, row in bm.iterrows():
        slot     = int(row["curve_slot"])
        ptype    = row.get("path_type", "?")
        total_l  = row.get("path_total_length_m", float('nan'))
        p90      = row.get("p90_mm", float('nan'))
        lt3      = row.get("ratio_lt_3mm", 0.0) * 100.0
        lt5      = row.get("ratio_lt_5mm", 0.0) * 100.0
        bias     = row.get("mean_signed_mm", float('nan'))
        passed   = int(row.get("pass_p90_lt_5mm", 0))

        results[slot] = {"p90": p90, "lt3": lt3/100.0, "lt5": lt5/100.0, "bias": bias}

        mark = "✓" if passed else "✗"
        line = (f"  curve_{slot:02d}  {ptype:>8}  {total_l:>8.3f}  "
                f"{p90:>8.3f}  {lt3:>6.1f}%  {lt5:>6.1f}%  "
                f"{bias:>+8.3f}mm  {mark:>5}")

        if prev_data and slot in prev_data:
            prev = prev_data[slot]
            delta_p90 = p90 - prev.get("p90", p90)
            sign = "↓" if delta_p90 < 0 else "↑"
            line += f"  p90{sign}{abs(delta_p90):.2f}mm"

        print(line)

    hr()

    # 汇总统计
    all_p90 = bm["p90_mm"].tolist()
    all_lt3 = (bm["ratio_lt_3mm"] * 100.0).tolist()
    pass_cnt = int(bm["pass_p90_lt_5mm"].sum())
    avg_p90  = float(np.mean(all_p90))
    avg_lt3  = float(np.mean(all_lt3))

    print(f"\n  【汇总】 avg_p90={avg_p90:.2f}mm  avg<3mm%={avg_lt3:.1f}%  "
          f"通过率={pass_cnt}/{len(bm)}")

    # 目标判断
    target_lt3 = 65.0
    target_p90 = 5.0
    print(f"\n  目标: avg<3mm% > {target_lt3:.0f}%  {pass_mark(avg_lt3, target_lt3, False)}  |  "
          f"全部 p90 < {target_p90:.0f}mm  {'✓' if pass_cnt == len(bm) else '✗'}")

    return results


# ─────────────────── 层2：单曲线深析 ───────────────────
def layer2_curve(curve_tag: str, latest_dir: Path):
    metrics_file = latest_dir / f"{curve_tag}_metrics.csv"
    samples_file = latest_dir / f"{curve_tag}_samples.csv"

    if not metrics_file.exists():
        print(f"[错误] 找不到指标文件: {metrics_file}")
        return

    m = pd.read_csv(metrics_file)
    subsec(f"层2 · {curve_tag} 指标详情")

    ptype    = m["path_type"].iloc[0] if "path_type" in m.columns else "?"
    total_l  = m["path_total_length_m"].iloc[0]
    n        = int(m["n_samples"].iloc[0])

    p50_mm   = m["p50_mm"].iloc[0]
    p90_mm   = m["p90_mm"].iloc[0]
    p95_mm   = m["p95_mm"].iloc[0]
    max_mm   = m["max_mm"].iloc[0]
    mean_mm  = m["mean_mm"].iloc[0]
    lt3      = m["ratio_lt_3mm"].iloc[0] * 100.0
    lt5      = m["ratio_lt_5mm"].iloc[0] * 100.0
    bias     = m["mean_signed_mm"].iloc[0]
    std_mm   = m["std_mm"].iloc[0]
    s1       = m["s1_mean_mm"].iloc[0]
    s2       = m["s2_mean_mm"].iloc[0]
    s3       = m["s3_mean_mm"].iloc[0]
    s4       = m["s4_mean_mm"].iloc[0]

    print(f"  路径类型     : {ptype}")
    print(f"  路径总长     : {total_l:.3f} m")
    print(f"  样本数       : {n}  (采样间隔 ~55ms @ 18Hz)")
    print()
    print("  ── 主指标（横向跟踪误差 cross_track = Frenet e_y）──")
    print(f"  P50          : {p50_mm:.3f} mm")
    print(f"  P90          : {p90_mm:.3f} mm   {pass_mark(p90_mm, 5.0)}")
    print(f"  P95          : {p95_mm:.3f} mm")
    print(f"  MAX          : {max_mm:.3f} mm")
    print(f"  MEAN(abs)    : {mean_mm:.3f} mm")
    print(f"  <3mm%        : {lt3:.1f}%   {pass_mark(lt3, 65.0, False)}")
    print(f"  <5mm%        : {lt5:.1f}%")
    print()
    print(f"  偏向(带符号) : {bias:+.3f} mm  (正=持续偏右，负=持续偏左)")
    print(f"  标准差       : {std_mm:.3f} mm  (位置噪声水平)")
    print()
    print(f"  ── 路径段分析（S1-S4，各 25% 弧长）──")
    print(f"    S1  0%~25%  : {s1:.3f} mm")
    print(f"    S2 25%~50%  : {s2:.3f} mm")
    print(f"    S3 50%~75%  : {s3:.3f} mm")
    print(f"    S4 75%~100% : {s4:.3f} mm")
    sec_max = max(s1, s2, s3, s4)
    sec_min = min(s1, s2, s3, s4)
    print(f"    段极差       : {sec_max - sec_min:.3f} mm  "
          f"{'（均匀）' if (sec_max - sec_min) < 1.0 else '⚠ 不均匀，检查入口/出口段'}")

    # 配置参数
    print()
    print("  配置参数：")
    for col in ["v_max", "omega_max", "lookahead_dist", "lookahead_time",
                "feedback_limit_ratio", "feedback_min_limit",
                "enable_integral", "q1_lqr", "q2_lqr"]:
        if col in m.columns:
            print(f"    {col:30s}: {m[col].iloc[0]}")

    # 样本深析
    if not samples_file.exists():
        print("\n  [提示] 样本文件不存在，跳过样本分析")
        return

    df = pd.read_csv(samples_file)
    layer3_samples(df, curve_tag)


# ─────────────────── 层3：样本信号分析 ───────────────────
def layer3_samples(df: pd.DataFrame, curve_tag: str):
    subsec(f"层3 · {curve_tag} 样本信号分析")

    n = len(df)
    if n == 0:
        print("  [空] 无样本数据")
        return

    ct_mm   = df["cross_track_mm"]
    ct_abs  = ct_mm.abs()

    print(f"  样本数      : {n}")
    print(f"  时长        : {df['t_s'].max():.1f} s")
    print()
    print("  ── 横向误差 cross_track_mm（主指标）──")
    print(f"    P50         : {np.percentile(ct_abs, 50):.3f} mm")
    print(f"    P90         : {np.percentile(ct_abs, 90):.3f} mm  {pass_mark(np.percentile(ct_abs, 90), 5.0)}")
    print(f"    <3mm%       : {(ct_abs < 3.0).sum()/n*100:.1f}%  {pass_mark((ct_abs < 3.0).sum()/n*100, 65.0, False)}")
    pct_right = (ct_mm > 0).sum() / n * 100.0
    pct_left  = (ct_mm < 0).sum() / n * 100.0
    print(f"    均值(带符号): {ct_mm.mean():+.3f} mm  （{'偏右' if ct_mm.mean() > 0 else '偏左'}）")
    print(f"    偏右比例    : {pct_right:.1f}%  | 偏左比例: {pct_left:.1f}%")
    print(f"    标准差      : {ct_mm.std():.3f} mm  （位置噪声水平）")
    print()

    # 路径进度段分析
    if "path_progress" in df.columns:
        print("  ── 路径进度段分析（各 25% 弧长）──")
        prog = df["path_progress"]
        for s_lo, s_hi, s_name in [(0, 0.25, "S1  0%~25%"),
                                    (0.25, 0.50, "S2 25%~50%"),
                                    (0.50, 0.75, "S3 50%~75%"),
                                    (0.75, 1.01, "S4 75%~100%")]:
            mask = (prog >= s_lo) & (prog < s_hi)
            if mask.sum() == 0:
                print(f"    {s_name}: (无样本)")
                continue
            s_p90  = np.percentile(ct_abs[mask], 90)
            s_mean = ct_abs[mask].mean()
            s_bias = ct_mm[mask].mean()
            print(f"    {s_name}: mean|e|={s_mean:.3f}mm  P90={s_p90:.3f}mm  bias={s_bias:+.3f}mm")
        print()

    # 前馈 vs 反馈量级
    if "omega_ff_rps" in df.columns and "omega_correction_rps" in df.columns:
        ff_mean  = df["omega_ff_rps"].abs().mean()
        fb_mean  = df["omega_correction_rps"].abs().mean()
        if ff_mean > 1e-6:
            fb_ff_ratio = fb_mean / ff_mean * 100.0
        else:
            fb_ff_ratio = float('nan')
        print("  ── 前馈/反馈量级 ──")
        print(f"    |ω_ff| 均值 : {ff_mean:.4f} rad/s")
        print(f"    |ω_fb| 均值 : {fb_mean:.4f} rad/s  ({fb_ff_ratio:.1f}% of ff)")

    # 反馈饱和率
    if "omega_correction_rps" in df.columns and "feedback_limit_rps" in df.columns:
        fb_out = df["omega_correction_rps"].abs()
        fb_lim = df["feedback_limit_rps"].abs()
        sat_mask = fb_out >= fb_lim * 0.99
        sat_rate = sat_mask.sum() / n * 100.0
        fb_lim_med = float(fb_lim.median())
        print()
        print(f"  反馈饱和率（|ω_fb| ≥ limit）：{sat_rate:.1f}%  "
              f"{'⚠ >30% 反馈被钳制' if sat_rate > 30 else '✓ 线性区为主'}")
        print(f"  feedback_limit 中值：{fb_lim_med*1000:.2f} mrad/s")

    # 角速度平滑度
    if "angular_cmd_rps" in df.columns and "t_s" in df.columns and n > 2:
        omega_arr = df["angular_cmd_rps"].values
        t_arr     = df["t_s"].values
        dt_med    = float(np.median(np.diff(t_arr)))
        d_omega   = np.diff(omega_arr)
        omega_dot = d_omega / dt_med

        omega_dot_max_cfg = 0.5  # rad/s²（yaml 配置值）
        clip_pct = np.sum(np.abs(omega_dot) >= omega_dot_max_cfg * 0.99) / len(omega_dot) * 100.0

        # 换向频率
        if "omega_correction_rps" in df.columns:
            omega_corr_arr = df["omega_correction_rps"].values
            flip_n    = np.sum(np.diff(np.sign(omega_corr_arr)) != 0)
            flip_freq = flip_n / (t_arr[-1] - t_arr[0] + 1e-9)
        else:
            flip_freq = 0.0

        print()
        print("  ── 角速度平滑度 ──")
        print(f"    omega_dot 截断率（≥omega_dot_max）: {clip_pct:.1f}%")
        print(f"    反馈换向频率                      : {flip_freq:.2f} Hz  "
              f"{'⚠ >3Hz 高频抖动' if flip_freq > 3.0 else '✓ 低频换向'}")

        # ω 与 ω_ff 偏差分析
        if "omega_ff_rps" in df.columns:
            omega_ff2   = df["omega_ff_rps"].values
            ff_mean2    = float(np.abs(omega_ff2).mean())
            delta_ff    = omega_arr - omega_ff2
            near_ff_15pct = np.sum(np.abs(delta_ff) < 0.15 * (ff_mean2 + 1e-9)) / n * 100
            delta_p90_pct = float(np.percentile(np.abs(delta_ff), 90)) / (ff_mean2 + 1e-9) * 100
            print(f"    落在 ff±15% 内               : {near_ff_15pct:.1f}%  "
                  f"{'✓ 顺滑' if near_ff_15pct > 60 else '⚠ 偏离前馈较多'}")
            print(f"    |Δω| P90 / |ω_ff|            : {delta_p90_pct:.1f}%")

    # 航向误差
    if "e_theta_rad" in df.columns:
        eth = df["e_theta_rad"]
        print()
        print(f"  航向误差 e_theta：")
        print(f"    |均值|  : {eth.abs().mean():.4f} rad  ({math.degrees(eth.abs().mean()):.2f}°)")
        print(f"    标准差  : {eth.std():.4f} rad")

    # 积分状态
    if "integral_state" in df.columns and df["integral_state"].abs().max() > 1e-6:
        ist = df["integral_state"]
        print()
        print(f"  积分状态 integral_e_y_：")
        print(f"    均值(带符号): {ist.mean():+.6f} m·s")
        print(f"    最大绝对值  : {ist.abs().max():.6f} m·s")
        if ist.abs().max() > 0.01:
            print("    ⚠ 积分量较大，注意 windup（检查 integral_max 设置）")

    # 双路径反馈分解（移植自圆弧控制器）
    if "omega_fb_raw_rps" in df.columns:
        fb_raw = df["omega_fb_raw_rps"]
        print()
        print("  ── 双路径反馈分解 ──")
        print(f"    原始 LQR 反馈 (tanh前) 均值: {fb_raw.abs().mean()*1000:.2f} mrad/s")
    if "omega_i_rps" in df.columns and df["omega_i_rps"].abs().max() > 1e-6:
        oi = df["omega_i_rps"]
        print(f"    积分输出 ω_i       均值    : {oi.abs().mean()*1000:.2f} mrad/s  "
              f"max={oi.abs().max()*1000:.2f} mrad/s")
    if "omega_before_limits_rps" in df.columns:
        obl = df["omega_before_limits_rps"]
        print(f"    限幅前总量 均值             : {obl.abs().mean()*1000:.2f} mrad/s  "
              f"P90={float(np.percentile(obl.abs(), 90))*1000:.2f} mrad/s")

    # 打印状态统计
    if "is_printing" in df.columns:
        n_print = int(df["is_printing"].sum())
        n_total = len(df)
        print()
        print(f"  打印状态：{n_print}/{n_total} 周期处于打印中 "
              f"({n_print/n_total*100:.1f}%)")

    # 增益信息
    if "k1" in df.columns and "k2" in df.columns:
        k1 = df["k1"].iloc[0]
        k2 = df["k2"].iloc[0]
        v  = df["linear_speed_mps"].mean() if "linear_speed_mps" in df.columns else 0.05
        print()
        print(f"  增益（K1={k1:.2f}, K2={k2:.2f}, v={v:.3f}m/s）：")
        zeta = zeta_curve(k1, k2, v)
        print(f"    阻尼比 ζ（参考）: {zeta:.3f}")


# ─────────────────── 历史对比 ───────────────────
def compare_history():
    section("历史批次对比")

    if not BASELINES:
        print("  [提示] 暂无历史数据，首次测试后将 batch 数据填入 BASELINES 字典")
        return

    for label, slots in BASELINES.items():
        p90s = [v["p90"] for v in slots.values()]
        lt3s = [v["lt3"] for v in slots.values()]
        avg_p90 = np.mean(p90s)
        avg_lt3 = np.mean(lt3s) * 100.0
        pass_cnt = sum(1 for p in p90s if p < 5.0)
        print(f"  {label}: avg_p90={avg_p90:.2f}mm  avg<3mm%={avg_lt3:.1f}%  pass={pass_cnt}/{len(slots)}")


# ─────────────────── 诊断建议 ───────────────────
def print_diagnosis(bm: pd.DataFrame):
    section("诊断建议")

    if bm is None or len(bm) == 0:
        print("  无数据，无法诊断")
        return

    all_p90  = bm["p90_mm"].tolist()
    avg_p90  = float(np.mean(all_p90))
    avg_lt3  = float((bm["ratio_lt_3mm"] * 100.0).mean())
    pass_cnt = int(bm["pass_p90_lt_5mm"].sum())

    if avg_p90 > 10.0:
        print("  🔴 p90 均值 > 10mm：精度极差，首先检查：")
        print("     1. 路径点密度是否足够（3mm 间距最佳）")
        print("     2. v_max 是否在 0.05 m/s 左右")
        print("     3. omega_max 是否足够（当前 0.3 rad/s）")
        print("     4. 曲率计算是否正确（三点法对点密度敏感）")
    elif avg_p90 > 5.0:
        print("  🟡 p90 均值 5~10mm：精度不足，建议分析：")
        print("     1. 查 mean_signed_mm：若系统性偏向 > 2mm，开启积分")
        print("     2. 查反馈饱和率：若 > 30%，适当放宽 feedback_limit_ratio")
        print("     3. 查段分布：若某段显著差，检查入口对准和前馈精度")
        print("     4. 查 std_mm：若 > 2mm，增大 sg_window（滤波增强）")
    else:
        print(f"  ✓ p90 均值 {avg_p90:.2f}mm 已达标（<5mm）")

    if avg_lt3 < 65.0:
        print(f"\n  🟡 avg<3mm% = {avg_lt3:.1f}% < 65%，提升方向：")
        print("     - 优先改善 p90 最差的曲线段")
        print("     - std_mm > 1.5mm → 增大 sg_window（7→9）")
        print("     - 有系统性偏向 → 开启积分")
    else:
        print(f"\n  ✓ avg<3mm% = {avg_lt3:.1f}% 已达标（>65%）")

    if "mean_signed_mm" in bm.columns:
        mean_bias = bm["mean_signed_mm"].abs().mean()
        if mean_bias > 2.0:
            print(f"\n  ⚠ 系统偏向均值 {mean_bias:.2f}mm > 2mm：")
            print("     建议开启积分（enable_integral: true）")
            print("     Ki 初始值: 0.3，integral_max: 0.020，integral_decay: 0.99")


# ─────────────────── 主入口 ───────────────────
def main():
    parser = argparse.ArgumentParser(
        description="曲线跟踪控制器数据分析脚本",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--curve", metavar="NN",
                        help="深析指定曲线（如 --curve 01）")
    parser.add_argument("--compare", action="store_true",
                        help="输出全历史批次对比")
    parser.add_argument("--all-deep", action="store_true",
                        help="全曲线深度分析（每曲线均执行层3）")
    args = parser.parse_args()

    # 确定数据目录
    ws_root = os.environ.get("XLINE_WS_ROOT", "")
    if ws_root:
        data_dir = Path(ws_root) / "curve_tracking_latest"
    else:
        data_dir = DATA_DIR

    if args.compare:
        compare_history()
        return

    # 检查数据目录
    if not data_dir.exists():
        print(f"[错误] 数据目录不存在: {data_dir}")
        print("[提示] 请先运行曲线跟踪测试，数据将自动保存到 curve_tracking_latest/")
        sys.exit(1)

    batch_file = data_dir / "batch_metrics.csv"
    if not batch_file.exists():
        print(f"[错误] 批次文件不存在: {batch_file}")
        print("[提示] 确认控制器已完成路径并导出数据")
        sys.exit(1)

    bm = pd.read_csv(batch_file)
    if len(bm) == 0:
        print("[错误] 批次文件为空")
        sys.exit(1)

    # 层1：批次概览
    layer1_batch(bm)

    # 诊断建议
    print_diagnosis(bm)

    # 层2/3：单曲线深析
    if args.curve:
        tag = f"curve_{args.curve.zfill(2)}"
        print()
        layer2_curve(tag, data_dir)
    elif args.all_deep:
        slots = bm["curve_slot"].tolist()
        for s in sorted(set(slots)):
            tag = f"curve_{int(s):02d}"
            print()
            layer2_curve(tag, data_dir)
    else:
        # 默认：显示最差曲线的层2（按 p90 排序）
        worst_slot = int(bm.loc[bm["p90_mm"].idxmax(), "curve_slot"])
        print(f"\n[自动深析最差曲线: curve_{worst_slot:02d}]")
        layer2_curve(f"curve_{worst_slot:02d}", data_dir)


if __name__ == "__main__":
    main()
