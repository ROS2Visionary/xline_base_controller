#!/usr/bin/env python3
"""
circle_tracking_latest 综合诊断脚本

用法（从项目根目录运行）:
  python3 circle_tuning/analyze.py                  # 分析最新批次
  python3 circle_tuning/analyze.py --circle 01      # 指定圆弧样本深层分析
  python3 circle_tuning/analyze.py --compare        # 输出全历史对比
  python3 circle_tuning/analyze.py --all-deep       # 全圆弧三层深度分析
"""

import sys, os, math, argparse
import pandas as pd
import numpy as np
from pathlib import Path

# ─────────────────── 配置 ───────────────────
DATA_DIR  = Path(__file__).parent.parent / "circle_tracking_latest"
HIST_FILE = Path(__file__).parent / "circle_optimization_history.md"

# 历史批次基准值（每次测试后手动填入）
# 格式: {"C<批次号>_<batch_id>": {圆弧槽: {lt3, lt5, p90, radial_mean}}}
BASELINES = {
    # 示例（首次测试后填入）：
    # "C00_<batch_id>": {
    #     1: {"lt3": 0.0, "lt5": 0.0, "p90": 0.0, "radial": 0.0},
    # },
}

# ─────────────────── 工具函数 ───────────────────
def hr(char="─", n=78): print(char * n)
def section(title): hr("═"); print(f"  {title}"); hr("═")
def subsec(title):  hr("─"); print(f"  {title}"); hr("─")

def zeta_circle(k1, k2, v, radius):
    """
    圆弧 LQR 阻尼比（近似）
    ζ = K2 / (2·√(v·K1))
    注：圆弧有前馈，K1/K2 实际被 feedback_limit 压制；此值供参考
    """
    if v <= 0 or k1 <= 0: return float('nan')
    return k2 / (2.0 * math.sqrt(v * k1))

def pass_mark(val, threshold, lower_is_better=True):
    """打印通过/失败标记"""
    if lower_is_better:
        return "✓" if val < threshold else "✗"
    else:
        return "✓" if val >= threshold else "✗"

def get_radial_col(row_or_df, col_radial, col_fallback):
    """优先取径向误差列，若不存在则 fallback 到控制误差列（兼容旧数据）"""
    if isinstance(row_or_df, pd.Series):
        if col_radial in row_or_df.index and not pd.isna(row_or_df.get(col_radial, float('nan'))):
            return row_or_df[col_radial]
        return row_or_df.get(col_fallback, float('nan'))
    else:
        if col_radial in row_or_df.columns:
            return row_or_df[col_radial]
        return row_or_df[col_fallback]

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
    hdr = f"  {'圆弧':>6}  {'半径(m)':>8}  {'类型':>6}  "
    hdr += f"{'p90(mm)':>8}  {'<3mm%':>7}  {'<5mm%':>7}  "
    hdr += f"{'径向偏差(mm)':>12}  {'通过':>5}"
    if prev_label:
        hdr += f"  {'vs父批次':>10}"
    print(hdr)
    hr()

    results = {}
    for _, row in bm.iterrows():
        slot    = int(row["circle_slot"])
        radius  = row.get("circle_radius_m", float('nan'))
        ptype   = row.get("path_type", "?")
        # 优先使用径向误差（几何精度），向后兼容使用控制误差
        p90     = get_radial_col(row, "p90_radial_mm", "p90_mm")
        lt3     = get_radial_col(row, "ratio_lt_3mm_radial", "ratio_lt_3mm") * 100.0
        lt5     = get_radial_col(row, "ratio_lt_5mm_radial", "ratio_lt_5mm") * 100.0
        radial  = row.get("mean_radial_mm", float('nan'))
        passed  = int(row.get("pass_p90_lt_5mm", 0))

        results[slot] = {"p90": p90, "lt3": lt3/100.0, "lt5": lt5/100.0, "radial": radial}

        mark = "✓" if passed else "✗"
        line = (f"  circle_{slot:02d}  {radius:>8.3f}  {ptype:>6}  "
                f"{p90:>8.3f}  {lt3:>6.1f}%  {lt5:>6.1f}%  "
                f"{radial:>+10.3f}mm  {mark:>5}")

        if prev_data and slot in prev_data:
            prev = prev_data[slot]
            delta_p90 = p90 - prev.get("p90", p90)
            sign = "↓" if delta_p90 < 0 else "↑"
            line += f"  p90{sign}{abs(delta_p90):.2f}mm"

        print(line)

    hr()

    # 半径-精度趋势（若有多个不同半径）
    if "circle_radius_m" in bm.columns:
        radii = bm["circle_radius_m"].unique()
        if len(radii) > 1:
            print("\n  半径-精度趋势（按半径排序，主指标=径向误差）：")
            bm_sorted = bm.sort_values("circle_radius_m")
            for _, row in bm_sorted.iterrows():
                r    = row["circle_radius_m"]
                p90  = get_radial_col(row, "p90_radial_mm", "p90_mm")
                lt3  = get_radial_col(row, "ratio_lt_3mm_radial", "ratio_lt_3mm") * 100.0
                v_actual = row.get("v_max", 0.07)
                ff   = v_actual / r if r > 0 else 0
                fb_ratio_after = row.get("feedback_ratio_after", 0.05)
                lim_after = ff * fb_ratio_after * 1000
                mark = "✓" if p90 < 5.0 else "✗"
                print(f"    R={r:.2f}m  v={v_actual:.3f}m/s  ω_ff={ff:.3f}rad/s  "
                      f"lim_after={lim_after:.1f}mrad/s  "
                      f"p90={p90:.2f}mm {mark}  <3mm%={lt3:.1f}%")

    # 汇总统计（优先使用径向误差）
    p90_col  = "p90_radial_mm" if "p90_radial_mm" in bm.columns else "p90_mm"
    lt3_col  = "ratio_lt_3mm_radial" if "ratio_lt_3mm_radial" in bm.columns else "ratio_lt_3mm"
    lt5_col  = "ratio_lt_5mm_radial" if "ratio_lt_5mm_radial" in bm.columns else "ratio_lt_5mm"
    all_p90  = bm[p90_col].tolist()
    all_lt3  = (bm[lt3_col] * 100.0).tolist()
    all_lt5  = (bm[lt5_col] * 100.0).tolist()
    pass_cnt = int(bm["pass_p90_lt_5mm"].sum())
    avg_lt3  = float(np.mean(all_lt3))
    avg_p90  = float(np.mean(all_p90))

    print(f"\n  【汇总】 avg_p90={avg_p90:.2f}mm  avg<3mm%={avg_lt3:.1f}%  "
          f"通过率={pass_cnt}/{len(bm)}")

    # 目标判断
    target_lt3 = 65.0
    target_p90 = 5.0
    print(f"\n  目标: avg<3mm% > {target_lt3:.0f}%  {pass_mark(avg_lt3, target_lt3, False)}  |  "
          f"全部 p90 < {target_p90:.0f}mm  {'✓' if pass_cnt == len(bm) else '✗'}")

    return results


# ─────────────────── 层2：单圆弧深析 ───────────────────
def layer2_circle(circle_tag: str, latest_dir: Path):
    metrics_file = latest_dir / f"{circle_tag}_metrics.csv"
    samples_file = latest_dir / f"{circle_tag}_samples.csv"

    if not metrics_file.exists():
        print(f"[错误] 找不到指标文件: {metrics_file}")
        return

    m = pd.read_csv(metrics_file)
    subsec(f"层2 · {circle_tag} 指标详情")

    radius   = m["circle_radius_m"].iloc[0]
    ptype    = m["path_type"].iloc[0] if "path_type" in m.columns else "?"
    total_a  = m["total_angle_rad"].iloc[0]
    n        = int(m["n_samples"].iloc[0])
    center_x = m["circle_center_x_m"].iloc[0] if "circle_center_x_m" in m.columns else float('nan')
    center_y = m["circle_center_y_m"].iloc[0] if "circle_center_y_m" in m.columns else float('nan')

    # 主指标（径向几何误差，向后兼容取控制误差）
    p90_rad  = get_radial_col(m.iloc[0], "p90_radial_mm", "p90_mm")
    p50_rad  = get_radial_col(m.iloc[0], "p50_radial_mm", "p50_mm")
    lt3_rad  = get_radial_col(m.iloc[0], "ratio_lt_3mm_radial", "ratio_lt_3mm") * 100.0
    lt5_rad  = get_radial_col(m.iloc[0], "ratio_lt_5mm_radial", "ratio_lt_5mm") * 100.0

    # 控制误差（参考）
    p90_ctrl = m["p90_mm"].iloc[0]
    lt3_ctrl = m["ratio_lt_3mm"].iloc[0] * 100.0
    p95      = m["p95_mm"].iloc[0]
    max_mm   = m["max_mm"].iloc[0]
    mean_mm  = m["mean_mm"].iloc[0]

    radial   = m["mean_radial_mm"].iloc[0]
    std_rad  = m["std_radial_mm"].iloc[0]

    print(f"  半径       : {radius:.3f} m")
    print(f"  类型       : {ptype}  (总角度={math.degrees(total_a):.1f}°)")
    print(f"  样本数     : {n}  (采样间隔 50ms)")
    if math.isfinite(center_x):
        print(f"  圆心坐标   : ({center_x:.4f}, {center_y:.4f}) m")
    print()
    print("  ── 主指标（径向几何误差 = dist(robot,center) - R）──")
    print(f"  P50 径向   : {p50_rad:.3f} mm")
    print(f"  P90 径向   : {p90_rad:.3f} mm   {pass_mark(p90_rad, 5.0)}")
    print(f"  <3mm% 径向 : {lt3_rad:.1f}%   {pass_mark(lt3_rad, 65.0, False)}")
    print(f"  <5mm% 径向 : {lt5_rad:.1f}%")
    print()
    print("  ── 参考指标（控制误差 = e_y at lookahead）──")
    print(f"  P90 控制   : {p90_ctrl:.3f} mm")
    print(f"  P95 控制   : {p95:.3f} mm")
    print(f"  MAX 控制   : {max_mm:.3f} mm")
    print(f"  MEAN 控制  : {mean_mm:.3f} mm")
    print(f"  <3mm% 控制 : {lt3_ctrl:.1f}%")
    print()
    print(f"  径向系统偏差: {radial:+.3f} mm  (正=持续偏外，负=持续偏内)")
    print(f"  径向标准差  : {std_rad:.3f} mm  (波动幅度)")

    # 象限分析（来自 metrics 的预计算值，基于径向误差）
    if all(c in m.columns for c in ["q1_mean_mm", "q2_mean_mm", "q3_mean_mm", "q4_mean_mm"]):
        q1 = m["q1_mean_mm"].iloc[0]
        q2 = m["q2_mean_mm"].iloc[0]
        q3 = m["q3_mean_mm"].iloc[0]
        q4 = m["q4_mean_mm"].iloc[0]
        print(f"\n  象限径向误差(mean|radial_err|):")
        print(f"    Q1 (0°~90°)   : {q1:.3f} mm")
        print(f"    Q2 (90°~180°) : {q2:.3f} mm")
        print(f"    Q3 (180°~270°): {q3:.3f} mm")
        print(f"    Q4 (270°~360°): {q4:.3f} mm")
        q_max = max(q1, q2, q3, q4)
        q_min = min(q1, q2, q3, q4)
        print(f"    象限极差      : {q_max - q_min:.3f} mm  "
              f"{'（均匀）' if (q_max - q_min) < 1.0 else '⚠ 不均匀，检查入圆/出圆条件'}")

    # 配置参数
    print()
    print("  配置参数：")
    for col in ["v_max", "omega_max", "lookahead_dist", "lookahead_time",
                "feedback_ratio_before", "feedback_ratio_after",
                "enable_integral", "q1_lqr", "q2_lqr"]:
        if col in m.columns:
            print(f"    {col:30s}: {m[col].iloc[0]}")

    # 样本深析
    if not samples_file.exists():
        print("\n  [提示] 样本文件不存在，跳过样本分析")
        return

    df = pd.read_csv(samples_file)
    layer3_samples(df, circle_tag)


def layer3_samples(df: pd.DataFrame, circle_tag: str):
    subsec(f"层3 · {circle_tag} 样本信号分析")

    n = len(df)
    if n == 0:
        print("  [空] 无样本数据")
        return

    # 径向误差为主
    ey_mm = df["cross_track_mm"]
    print(f"  样本数      : {n}")
    print(f"  时长        : {df['t_s'].max():.1f} s")
    print()

    if "radial_err_mm" in df.columns:
        rad = df["radial_err_mm"]
        rad_abs = rad.abs()
        pct_outside = (rad > 0).sum() / n * 100.0
        pct_inside  = (rad < 0).sum() / n * 100.0
        print("  ── 径向误差（主指标，几何精度）──")
        print(f"    P50         : {np.percentile(rad_abs, 50):.3f} mm")
        print(f"    P90         : {np.percentile(rad_abs, 90):.3f} mm  {pass_mark(np.percentile(rad_abs, 90), 5.0)}")
        print(f"    <3mm%       : {(rad_abs < 3.0).sum()/n*100:.1f}%  {pass_mark((rad_abs < 3.0).sum()/n*100, 65.0, False)}")
        print(f"    均值(带符号): {rad.mean():+.3f} mm  （正=持续偏外，负=持续偏内）")
        print(f"    偏外比例    : {pct_outside:.1f}%  | 偏内比例: {pct_inside:.1f}%")
        print()

        # 基于 real_angle_rad 的象限分析（优先），fallback 到 accum_angle_rad
        angle_col = "real_angle_rad" if "real_angle_rad" in df.columns else "accum_angle_rad"
        angle_label = "真实几何角" if angle_col == "real_angle_rad" else "偏航积分角（近似）"
        print(f"  ── 象限径向误差分析（{angle_label}）──")
        quarters = [(0, math.pi*0.5, "Q1  0°~ 90°"),
                    (math.pi*0.5, math.pi, "Q2 90°~180°"),
                    (math.pi, math.pi*1.5, "Q3 180°~270°"),
                    (math.pi*1.5, math.pi*2, "Q4 270°~360°")]
        ang = df[angle_col]
        for q_lo, q_hi, q_name in quarters:
            mask = (ang >= q_lo) & (ang < q_hi)
            if mask.sum() == 0:
                print(f"    {q_name}: (无样本)")
                continue
            q_p90 = np.percentile(rad_abs[mask], 90)
            q_mean = rad_abs[mask].mean()
            q_bias = rad[mask].mean()
            print(f"    {q_name}: mean|r|={q_mean:.3f}mm  P90={q_p90:.3f}mm  bias={q_bias:+.3f}mm")
        print()

    print("  ── 横向控制误差 cross_track_mm（Frenet e_y，参考）──")
    print(f"    均值(带符号): {ey_mm.mean():+.3f} mm  "
          f"（{'偏左' if ey_mm.mean() < 0 else '偏右'}）")
    print(f"    标准差      : {ey_mm.std():.3f} mm  （位置噪声水平）")

    # 反馈饱和分析（C05+双路架构：比例路径饱和 vs 积分贡献）
    if "omega_fb_raw_rps" in df.columns and "feedback_limit_rps" in df.columns:
        fb_raw  = df["omega_fb_raw_rps"].values
        fb_lim  = df["feedback_limit_rps"].values
        # 比例路径实际输出（tanh软饱和）
        prop_out = np.tanh(fb_raw / np.where(fb_lim > 1e-9, fb_lim, 1e-9)) * fb_lim
        # 积分路径贡献（omega_correction - omega_prop）
        if "omega_correction_rps" in df.columns:
            omega_int_est = df["omega_correction_rps"].values - prop_out
        else:
            omega_int_est = np.zeros(n)
        # 比例路径软饱和率：|raw| > 2×limit 时 tanh(2)=0.96，基本已饱和
        prop_sat_mask = np.abs(fb_raw) > 2.0 * np.where(fb_lim > 1e-9, fb_lim, 1e-9)
        prop_sat_rate = prop_sat_mask.sum() / n * 100.0
        int_contrib_mean = np.abs(omega_int_est).mean() * 1000.0  # mrad/s
        ff_mean_local = df["omega_ff_rps"].abs().mean() if "omega_ff_rps" in df.columns else 1.0
        int_contrib_pct = (np.abs(omega_int_est).mean() / (ff_mean_local + 1e-9)) * 100.0
        print()
        print(f"  比例路径软饱和率（|raw|>2×limit，tanh≥0.96）：{prop_sat_rate:.1f}%")
        if prop_sat_rate > 60.0:
            print("    ⚠ 比例路径基本饱和：K1偏大或limit_ratio偏小，纠偏效果退化")
        elif prop_sat_rate < 20.0:
            print("    ✓ 比例路径工作在线性区（tanh比例控制有效）")
        print(f"  积分路径贡献（估算）：均值={int_contrib_mean:.1f}mrad/s = {int_contrib_pct:.1f}%ω_ff")

    # 前馈 vs 反馈量级
    if "omega_ff_rps" in df.columns and "omega_correction_rps" in df.columns:
        ff_mean  = df["omega_ff_rps"].abs().mean()
        fb_mean  = df["omega_correction_rps"].abs().mean()
        if ff_mean > 1e-6:
            fb_ff_ratio = fb_mean / ff_mean * 100.0
        else:
            fb_ff_ratio = float('nan')
        print()
        print(f"  前馈/反馈量级：")
        print(f"    |ω_ff| 均值 : {ff_mean:.4f} rad/s")
        print(f"    |ω_fb| 均值 : {fb_mean:.4f} rad/s  ({fb_ff_ratio:.1f}% of ff)")

    # 喷墨前后精度对比
    if "is_printing" in df.columns:
        df_before = df[df["is_printing"] == 0]
        df_after  = df[df["is_printing"] == 1]
        if len(df_before) > 0 and len(df_after) > 0:
            p90_before = np.percentile(df_before["cross_track_mm"].abs(), 90)
            p90_after  = np.percentile(df_after["cross_track_mm"].abs(), 90)
            print()
            print(f"  喷墨前后精度对比：")
            print(f"    喷墨前 p90 : {p90_before:.3f} mm  (n={len(df_before)}, limit={df_before['feedback_limit_rps'].mean()*1000:.1f}mrad/s)")
            print(f"    喷墨后 p90 : {p90_after:.3f}  mm  (n={len(df_after)},  limit={df_after['feedback_limit_rps'].mean()*1000:.1f}mrad/s)")
            if p90_after > p90_before * 1.5:
                print("    ⚠ 喷墨后精度显著下降：feedback_limit 可能过紧（5% → 考虑 8%~10%）")

    # 积分状态分析
    if "integral_state" in df.columns and df["integral_state"].abs().max() > 1e-6:
        ist = df["integral_state"]
        print()
        print(f"  积分状态 integral_e_y_：")
        print(f"    均值(带符号): {ist.mean():+.6f} m·s")
        print(f"    最大绝对值  : {ist.abs().max():.6f} m·s")
        if ist.abs().max() > 0.01:
            print("    ⚠ 积分量较大，注意 windup（检查 integral_max 设置）")

    # ── 角速度连续性与平滑度分析（线条顺滑性）──
    if "angular_cmd_rps" in df.columns and "t_s" in df.columns and n > 2:
        omega_arr  = df["angular_cmd_rps"].values
        t_arr      = df["t_s"].values
        dt_med     = float(np.median(np.diff(t_arr)))
        d_omega    = np.diff(omega_arr)
        omega_dot  = d_omega / dt_med  # rad/s²

        omega_dot_max_cfg = 0.5  # rad/s²（yaml配置值）
        clip_pct  = np.sum(np.abs(omega_dot) >= omega_dot_max_cfg * 0.99) / len(omega_dot) * 100.0
        smooth_std = np.std(d_omega) * 1000  # mrad/s

        # bang-bang 换向特征
        omega_corr_arr = df["omega_correction_rps"].values if "omega_correction_rps" in df.columns else omega_arr
        flip_n    = np.sum(np.diff(np.sign(omega_corr_arr)) != 0)
        flip_freq = flip_n / (t_arr[-1] - t_arr[0] + 1e-9)

        # 平均单侧持续时间
        fb_lim_arr = df["feedback_limit_rps"].values if "feedback_limit_rps" in df.columns else np.ones(n) * 0.02
        fb_lim_med = float(np.median(fb_lim_arr))
        at_pos = omega_corr_arr >= fb_lim_med * 0.99
        at_neg = omega_corr_arr <= -fb_lim_med * 0.99
        sign_a = np.where(at_pos, 1, np.where(at_neg, -1, 0))
        runs = []
        cv, cl = sign_a[0], 1
        for i in range(1, len(sign_a)):
            if sign_a[i] == cv and cv != 0:
                cl += 1
            else:
                if cv != 0:
                    runs.append(cl)
                cv, cl = sign_a[i], 1
        if cv != 0:
            runs.append(cl)
        avg_run_ms = float(np.mean(runs)) * dt_med * 1000 if runs else 0

        # 路径侧向波动简化估计
        flip_idxs = np.where(np.diff(np.sign(omega_corr_arr)) != 0)[0]
        if len(flip_idxs) > 1:
            T_half = float(np.mean(np.diff(t_arr[flip_idxs])))
            v_mean = df["linear_speed_mps"].mean() if "linear_speed_mps" in df.columns else 0.06
            lat_osc_mm = v_mean * fb_lim_med * (T_half ** 2) / 2 * 1000
        else:
            lat_osc_mm = 0.0

        # ── 核心平滑度指标：omega 落在 ff 附近的比例 ──
        omega_arr2  = df["angular_cmd_rps"].values
        omega_ff2   = df["omega_ff_rps"].values if "omega_ff_rps" in df.columns else np.ones(n) * 0.1
        ff_mean2    = float(np.abs(omega_ff2).mean())
        delta_ff    = omega_arr2 - omega_ff2
        near_ff_5pct  = np.sum(np.abs(delta_ff) < 0.05 * ff_mean2) / n * 100
        near_ff_15pct = np.sum(np.abs(delta_ff) < 0.15 * ff_mean2) / n * 100
        near_ff_25pct = np.sum(np.abs(delta_ff) < 0.25 * ff_mean2) / n * 100
        delta_p50_pct = float(np.percentile(np.abs(delta_ff), 50)) / (ff_mean2 + 1e-9) * 100
        delta_p90_pct = float(np.percentile(np.abs(delta_ff), 90)) / (ff_mean2 + 1e-9) * 100

        # 控制模式判断：比例区比例
        # C05+双路架构：比例区判断基于 omega_fb_raw vs prop_limit（tanh线性区）
        fb_raw2     = df["omega_fb_raw_rps"].values if "omega_fb_raw_rps" in df.columns else np.zeros(n)
        fb_lim2     = df["feedback_limit_rps"].values if "feedback_limit_rps" in df.columns else np.ones(n)*0.05
        fb_lim_med2 = float(np.median(fb_lim2))
        # 比例区：|raw_fb| < limit（tanh输入<1，输出线性）；饱和区：|raw_fb| > 2×limit
        in_prop_zone = np.abs(fb_raw2) < fb_lim_med2
        prop_pct  = in_prop_zone.sum() / n * 100
        bang_pct  = (np.abs(fb_raw2) > 2.0 * fb_lim_med2).sum() / n * 100

        print()
        print("  ── 角速度连续性与平滑度──")
        print(f"    控制模式     : 比例线性区={prop_pct:.1f}%  软饱和区={100-prop_pct-bang_pct:.1f}%  硬饱和区≈{bang_pct:.1f}%  "
              f"{'✓ 比例为主' if prop_pct > 50 else ('⚠ 软饱和为主' if bang_pct < 30 else '⚠ 接近饱和')}")
        print(f"    |Δ from ff| P50/P90  : {delta_p50_pct:.1f}% / {delta_p90_pct:.1f}% of ω_ff")
        print(f"    落在ff±5%内  : {near_ff_5pct:.1f}%  "
              f"落在ff±15%内: {near_ff_15pct:.1f}%  "
              f"落在ff±25%内: {near_ff_25pct:.1f}%")
        smooth_target = near_ff_15pct > 60
        print(f"    平滑目标(ff±15%>60%) : {'✓ 达到' if smooth_target else '✗ 未达到，仍以bang-bang为主'}")
        print(f"    omega_dot_max截断率  : {clip_pct:.1f}%")
        print(f"    反馈换向频率         : {flip_freq:.2f} Hz  "
              f"{'⚠ >3Hz 高频抖动' if flip_freq > 3.0 else '✓ 低频换向'}")
        print(f"    路径侧向波动估计     : {lat_osc_mm:.2f} mm  "
              f"{'⚠ >1mm 线条可见摆动' if lat_osc_mm > 1.0 else '✓ <1mm 顺滑'}")

    # e_theta 分析
    if "e_theta_rad" in df.columns:
        eth = df["e_theta_rad"]
        print()
        print(f"  航向误差 e_theta：")
        print(f"    |均值|      : {eth.abs().mean():.4f} rad  ({math.degrees(eth.abs().mean()):.2f}°)")
        print(f"    标准差      : {eth.std():.4f} rad")

    # 增益信息
    if "k1" in df.columns and "k2" in df.columns:
        k1 = df["k1"].iloc[0]
        k2 = df["k2"].iloc[0]
        v  = df["linear_speed_mps"].mean() if "linear_speed_mps" in df.columns else 0.07
        r  = df["circle_radius_m"].mean()  if "circle_radius_m"  in df.columns else float('nan')
        print()
        print(f"  增益（K1={k1:.2f}, K2={k2:.2f}, v={v:.3f}m/s）：")
        if math.isfinite(r) and r > 0:
            omega_ff_theory = v / r
            print(f"    理论前馈   : v/R = {v:.3f}/{r:.3f} = {omega_ff_theory:.4f} rad/s")
        zeta = zeta_circle(k1, k2, v, r if math.isfinite(r) else 1.0)
        print(f"    阻尼比 ζ  : {zeta:.3f}  （参考值，圆弧有前馈所以意义与直线不同）")


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

    p90_col  = "p90_radial_mm" if "p90_radial_mm" in bm.columns else "p90_mm"
    lt3_col  = "ratio_lt_3mm_radial" if "ratio_lt_3mm_radial" in bm.columns else "ratio_lt_3mm"
    all_p90   = bm[p90_col].tolist()
    avg_p90   = float(np.mean(all_p90))
    avg_lt3   = float((bm[lt3_col] * 100.0).mean())
    pass_cnt  = int(bm["pass_p90_lt_5mm"].sum())

    if avg_p90 > 10.0:
        print("  🔴 p90 均值 > 10mm：精度极差，首先检查：")
        print("     1. 圆心坐标是否准确传入")
        print("     2. v_max 是否在 0.05~0.10 范围内")
        print("     3. omega_max 是否足够（当前 0.3 rad/s）")
    elif avg_p90 > 5.0:
        print("  🟡 p90 均值 5~10mm：精度不足，建议分析：")
        print("     1. 查 radial_err_mm 均值：若系统性偏内/偏外 > 2mm，开启积分")
        print("     2. 查反馈饱和率：若 > 30%，适当放宽 feedback_limit_ratio")
        print("     3. 查象限分布：若某象限显著差，检查入圆对准和前馈精度")
    else:
        print(f"  ✓ p90 均值 {avg_p90:.2f}mm 已达标（<5mm）")

    if avg_lt3 < 65.0:
        print(f"\n  🟡 avg<3mm% = {avg_lt3:.1f}% < 65%，提升方向：")
        print("     - 优先改善 p90 最差的圆弧")
        print("     - 检查噪声：e_y 标准差 > 1.5mm 说明位置噪声大，增大 sg_window")
    else:
        print(f"\n  ✓ avg<3mm% = {avg_lt3:.1f}% 已达标（>65%）")

    # 径向偏差诊断
    if "mean_radial_mm" in bm.columns:
        mean_rad = bm["mean_radial_mm"].abs().mean()
        if mean_rad > 2.0:
            print(f"\n  ⚠ 径向偏差均值 {mean_rad:.2f}mm > 2mm：")
            print("     系统性偏心，建议开启积分（enable_integral: true）")
            print("     Ki 建议初始值: 0.3~0.5，integral_max: 0.02")


# ─────────────────── 主入口 ───────────────────
def main():
    parser = argparse.ArgumentParser(
        description="圆弧跟踪控制器数据分析脚本",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--circle", metavar="NN",
                        help="深析指定圆弧（如 --circle 01）")
    parser.add_argument("--compare", action="store_true",
                        help="输出全历史批次对比")
    parser.add_argument("--all-deep", action="store_true",
                        help="全圆弧深度分析（每圆弧均执行层3）")
    args = parser.parse_args()

    # 确定数据目录
    ws_root = os.environ.get("XLINE_WS_ROOT", "")
    if ws_root:
        data_dir = Path(ws_root) / "circle_tracking_latest"
    else:
        data_dir = DATA_DIR

    if args.compare:
        compare_history()
        return

    # 检查数据目录
    if not data_dir.exists():
        print(f"[错误] 数据目录不存在: {data_dir}")
        print("[提示] 请先运行圆弧跟踪测试，数据将自动保存到 circle_tracking_latest/")
        sys.exit(1)

    batch_file = data_dir / "batch_metrics.csv"
    if not batch_file.exists():
        print(f"[错误] 批次文件不存在: {batch_file}")
        print("[提示] 确认控制器已完成至少一圈并导出数据")
        sys.exit(1)

    bm = pd.read_csv(batch_file)
    if len(bm) == 0:
        print("[错误] 批次文件为空")
        sys.exit(1)

    # 层1：批次概览
    layer1_batch(bm)

    # 诊断建议
    print_diagnosis(bm)

    # 层2/3：单圆弧深析
    if args.circle:
        tag = f"circle_{args.circle.zfill(2)}"
        print()
        layer2_circle(tag, data_dir)
    elif args.all_deep:
        slots = bm["circle_slot"].tolist()
        for s in sorted(set(slots)):
            tag = f"circle_{int(s):02d}"
            print()
            layer2_circle(tag, data_dir)
    else:
        # 默认：显示最差圆弧的层2（按径向 p90 排序）
        p90_col = "p90_radial_mm" if "p90_radial_mm" in bm.columns else "p90_mm"
        worst_slot = int(bm.loc[bm[p90_col].idxmax(), "circle_slot"])
        print(f"\n[自动深析最差圆弧: circle_{worst_slot:02d}]")
        layer2_circle(f"circle_{worst_slot:02d}", data_dir)


if __name__ == "__main__":
    main()
