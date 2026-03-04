#!/usr/bin/env python3
"""
line_tracking_latest 综合诊断脚本
基于 tuning_guide.md 三层漏斗分析法

用法（从项目根目录运行）:
  python3 lqr_tuning/analyze.py                  # 分析最新批次
  python3 lqr_tuning/analyze.py --path 05        # 指定路径样本深层分析
  python3 lqr_tuning/analyze.py --compare        # 输出全历史对比
  python3 lqr_tuning/analyze.py --all-deep       # 全路径三层深度分析
"""

import sys, os, math, argparse
import pandas as pd
import numpy as np
from pathlib import Path

# ─────────────────── 配置 ───────────────────
DATA_DIR  = Path(__file__).parent.parent / "line_tracking_latest"
HIST_FILE = Path(__file__).parent / "line_optimization_history.md"

# 历史批次基准值 {标签: {路径槽: {lt3, lt5, p90, drift}}}
BASELINES = {
    "B33_1772641030819": {
        1:{"lt3":.758,"lt5":.957,"p90":4.33,"drift":+3.08},
        2:{"lt3":.687,"lt5":.867,"p90":5.66,"drift":+0.16},
        3:{"lt3":.508,"lt5":.834,"p90":5.92,"drift":-2.28},
        4:{"lt3":.667,"lt5":.842,"p90":6.43,"drift":+0.38},
        5:{"lt3":.506,"lt5":.849,"p90":5.48,"drift":-0.15},
        6:{"lt3":.429,"lt5":.650,"p90":12.07,"drift":-4.14},
    },
    "B34_1772643134732": {
        1:{"lt3":.430,"lt5":.957,"p90":4.60},
        2:{"lt3":.431,"lt5":.723,"p90":6.88},
        3:{"lt3":.568,"lt5":.804,"p90":5.81},
        4:{"lt3":.766,"lt5":.904,"p90":4.92},
        5:{"lt3":.654,"lt5":.892,"p90":5.13},
        6:{"lt3":.488,"lt5":.709,"p90":9.79},
    },
    "B35_1772645333839": {
        1:{"lt3":.784,"lt5":.990,"p90":3.69},
        2:{"lt3":.628,"lt5":.874,"p90":5.41},
        3:{"lt3":.549,"lt5":.870,"p90":5.20},
        4:{"lt3":.736,"lt5":.876,"p90":5.54},
        5:{"lt3":.587,"lt5":.898,"p90":5.01},
        6:{"lt3":.443,"lt5":.657,"p90":10.37},
    },
    "B36_1772648450356": {
        1:{"lt3":.812,"lt5":.986,"p90":4.35},
        2:{"lt3":.622,"lt5":.941,"p90":4.53},
        3:{"lt3":.549,"lt5":.801,"p90":6.11},
        4:{"lt3":.773,"lt5":.922,"p90":4.44},
        5:{"lt3":.427,"lt5":.759,"p90":6.79},
        6:{"lt3":.448,"lt5":.612,"p90":11.76},
    },
    "B37_1772650459293": {
        1:{"lt3":.437,"lt5":.908,"p90":4.99},
        2:{"lt3":.525,"lt5":.774,"p90":6.86},
        3:{"lt3":.469,"lt5":.687,"p90":6.52},
        4:{"lt3":.737,"lt5":.890,"p90":5.31},
        5:{"lt3":.603,"lt5":.934,"p90":4.63},
        6:{"lt3":.391,"lt5":.759,"p90":6.05},
    },
    "B38_1772652009287": {
        1:{"lt3":.288,"lt5":.790,"p90":5.79},
        2:{"lt3":.421,"lt5":.729,"p90":7.17},
        3:{"lt3":.492,"lt5":.764,"p90":6.15},
        4:{"lt3":.708,"lt5":.900,"p90":4.99},
        5:{"lt3":.597,"lt5":.938,"p90":4.47},
        6:{"lt3":.502,"lt5":.791,"p90":8.93},
    },
}
# 锚定基准（B33，用于 vs B33 列）
B33 = BASELINES["B33_1772641030819"]

# ─────────────────── 工具函数 ───────────────────
def hr(char="─", n=78): print(char * n)
def section(title): hr("═"); print(f"  {title}"); hr("═")
def subsec(title):  hr("─"); print(f"  {title}"); hr("─")

def zeta(k1, k2, v):
    """计算阻尼比 ζ = K2 / (2√(v·K1))"""
    if v <= 0 or k1 <= 0: return float('nan')
    return k2 / (2 * math.sqrt(v * k1))

# ─────────────────── 层1：批次概览 ───────────────────
def layer1_batch(bm: pd.DataFrame) -> dict:
    opt_id   = bm["optimization_id"].iloc[0]
    batch_id = bm["batch_id"].iloc[0]
    parent_id = bm["parent_batch_id"].iloc[0] if "parent_batch_id" in bm.columns else "?"

    section(f"层1 · 批次概览  (batch={batch_id})")
    print(f"  优化ID      : {opt_id}")
    print(f"  父批次      : {parent_id}")

    # 找到父批次的历史基准
    prev_label = None
    for label in reversed(list(BASELINES.keys())):
        if str(parent_id) in label:
            prev_label = label
            break
    if prev_label is None:
        prev_label = "B33_1772641030819"
    prev = BASELINES[prev_label]

    passes    = int(bm["pass_p90_lt_5mm"].sum())
    avg_lt3   = bm["ratio_lt_3mm"].mean()
    avg_lt5   = bm["ratio_lt_5mm"].mean()
    avg_p90   = bm["p90_mm"].mean()
    b33_avg_lt3 = sum(B33[s]["lt3"] for s in B33) / 6
    b33_avg_p90 = sum(B33[s]["p90"] for s in B33) / 6
    prev_avg_lt3 = sum(prev[s]["lt3"] for s in prev) / 6
    prev_avg_p90 = sum(prev[s]["p90"] for s in prev) / 6

    print()
    print(f"  {'指标':<22} {'当前':>10}  {'vs B33':>10}  {'vs上批({})'.format(prev_label[:3]):>12}")
    print(f"  {'─'*22} {'─'*10}  {'─'*10}  {'─'*12}")
    print(f"  {'平均 <3mm%':<22} {avg_lt3*100:>9.1f}%  {(avg_lt3-b33_avg_lt3)*100:>+9.1f}pp  {(avg_lt3-prev_avg_lt3)*100:>+11.1f}pp")
    print(f"  {'平均 <5mm%':<22} {avg_lt5*100:>9.1f}%  {'─':>10}  {'─':>12}")
    print(f"  {'平均 p90':<22} {avg_p90:>9.2f}mm  {avg_p90-b33_avg_p90:>+9.2f}mm  {avg_p90-prev_avg_p90:>+11.2f}mm")
    print(f"  {'达标数(p90<5mm)':<22} {passes:>9}/6  {'─':>10}  {'─':>12}")

    print()
    print(f"  {'路径':<5} {'<3mm%':>7} {'ΔvsB33':>8} {'Δvs上批':>9} │ {'<5mm%':>7} {'p90':>8} {'Δp90vsB33':>11} {'状态':>5} {'长违3':>7} {'长违5':>7}")
    print(f"  {'─'*5} {'─'*7} {'─'*8} {'─'*9} ┼ {'─'*7} {'─'*8} {'─'*11} {'─'*5} {'─'*7} {'─'*7}")

    path_results = {}
    for _, row in bm.iterrows():
        s    = int(row["path_slot"])
        lt3  = row["ratio_lt_3mm"]; lt5 = row["ratio_lt_5mm"]; p90 = row["p90_mm"]
        d33  = (lt3 - B33[s]["lt3"]) * 100
        dprev= (lt3 - prev.get(s, B33[s])["lt3"]) * 100
        dp90 = p90 - B33[s]["p90"]
        pk   = "✓" if row["pass_p90_lt_5mm"] else "✗"
        breach3 = int(row["longest_breach_3_samples"])
        breach5 = int(row["longest_breach_5_samples"])
        drift   = row["drift_tail_minus_head_mm"]

        # 异常标志
        flags = []
        if lt3 < 0.50:   flags.append("<3mm%↓")
        if p90 > 8.0:    flags.append("p90↑↑")
        if breach3 > 40: flags.append("持续漂移")
        if breach5 > 30: flags.append("大振荡")

        flag_str = " ".join(flags)
        print(f"  {s:<5} {lt3*100:>6.1f}% {d33:>+7.1f}pp {dprev:>+8.1f}pp │ {lt5*100:>6.1f}% {p90:>7.2f}mm {dp90:>+10.2f}mm {pk:>5} {breach3:>7} {breach5:>7}  {flag_str}")

        path_results[s] = dict(lt3=lt3, lt5=lt5, p90=p90, drift=drift,
                               breach3=breach3, breach5=breach5,
                               d33=d33, dprev=dprev, passes=row["pass_p90_lt_5mm"])

    print()
    print(f"  {'均值':<5} {avg_lt3*100:>6.1f}% {(avg_lt3-b33_avg_lt3)*100:>+7.1f}pp {(avg_lt3-prev_avg_lt3)*100:>+8.1f}pp │ {avg_lt5*100:>6.1f}% {avg_p90:>7.2f}mm {avg_p90-b33_avg_p90:>+10.2f}mm {passes:>4}/6")

    return path_results

# ─────────────────── 层2：路径诊断 ───────────────────
def diagnose_path(s, row, samples: pd.DataFrame):
    """基于 tuning_guide 症状决策树自动诊断，返回问题列表和诊断数据字典。"""
    issues = []
    p90     = row["p90_mm"]
    lt3     = row["ratio_lt_3mm"]
    drift   = row["drift_tail_minus_head_mm"]
    breach5 = row["longest_breach_5_samples"]
    seg1_lt3 = row["seg1_ratio_lt_3mm"]
    seg3_lt3 = row["seg3_ratio_lt_3mm"]

    ct       = samples["cross_track_mm"]
    ct_mean  = ct.mean()

    # 角速度饱和率
    if "omega_before_limits" in samples.columns and "phase_max_w" in samples.columns:
        sat = (samples["omega_before_limits"].abs() > samples["phase_max_w"] * 0.95).mean()
    else:
        sat = 0.0

    # cancel_ratio 统计
    cancel_mean  = samples["cancel_ratio"].mean()  if "cancel_ratio" in samples.columns else 0
    cancel_heavy = (samples["cancel_ratio"] > 0.5).mean() if "cancel_ratio" in samples.columns else 0

    # K2 floor 激活率
    floor_pct = samples["k2_floor_active"].mean() if "k2_floor_active" in samples.columns else 0

    # 积分饱和（以 85%×integral_max 为阈值）
    intg_max          = samples["integral_state"].abs().max() if "integral_state" in samples.columns else 0
    integral_max_param = 0.015
    intg_saturation   = (samples["integral_state"].abs() > integral_max_param * 0.85).mean() \
                        if "integral_state" in samples.columns else 0

    # 阻尼比 ζ
    if "k1" in samples.columns and "k2" in samples.columns and "linear_speed_mps" in samples.columns:
        spd         = samples["linear_speed_mps"].replace(0, np.nan)
        zeta_series = samples["k2"] / (2 * (spd * samples["k1"]).pow(0.5))
        zeta_mean   = zeta_series.mean()
        underdamped_pct = (zeta_series < 0.5).mean()
    else:
        zeta_mean = float("nan"); underdamped_pct = 0

    # e_theta 滤波前后噪声
    if "e_theta_raw_rad" in samples.columns and "e_theta_rad" in samples.columns:
        etheta_noise     = (samples["e_theta_raw_rad"] - samples["e_theta_rad"]).abs()
        etheta_noise_std = etheta_noise.std()
        etheta_spike_pct = (etheta_noise > 0.02).mean()
    else:
        etheta_noise_std = 0; etheta_spike_pct = 0

    # 进入条件
    ct0 = ct.iloc[0]
    v0  = samples["linear_speed_mps"].iloc[0] if "linear_speed_mps" in samples.columns else 0

    row_diag = dict(
        p90=p90, lt3=lt3, drift=drift, breach5=breach5,
        seg1_lt3=seg1_lt3, seg3_lt3=seg3_lt3,
        sat=sat, cancel_mean=cancel_mean, cancel_heavy=cancel_heavy,
        floor_pct=floor_pct, intg_max=intg_max, intg_saturation=intg_saturation,
        zeta_mean=zeta_mean, underdamped_pct=underdamped_pct,
        etheta_noise_std=etheta_noise_std, etheta_spike_pct=etheta_spike_pct,
        ct0=ct0, v0=v0, ct_mean=ct_mean,
    )

    # ── 症状1：末段单向漂移 ──
    seg_diff = seg3_lt3 - seg1_lt3
    if abs(drift) > 2.0 and seg_diff < -0.15:
        if sat > 0.08:
            issues.append(("⚠ 症状1 末段漂移+角速度饱和",
                           f"drift={drift:+.1f}mm, sat={sat:.1%} → 减小K1或增大max_w"))
        elif intg_saturation < 0.30:
            issues.append(("⚠ 症状1 末段漂移+积分不足",
                           f"drift={drift:+.1f}mm, intg_sat={intg_saturation:.1%} → 考虑增大Ki或integral_max"))
        else:
            issues.append(("⚠ 症状1 末段漂移",
                           f"drift={drift:+.1f}mm → K1比例修正力不足"))

    # ── 症状2：振荡（p90高但漂移小）──
    if p90 > 5.5 and abs(drift) < 2.0:
        if cancel_heavy > 0.15:
            if floor_pct > 0.05:
                issues.append(("⚠ 症状2 振荡+floor常态激活",
                               f"cancel_heavy={cancel_heavy:.1%}, floor_active={floor_pct:.1%} → 增大K2或k2_min_floor_scale"))
            else:
                issues.append(("⚠ 症状2 振荡+cancel频发",
                               f"cancel_heavy={cancel_heavy:.1%}, ζ={zeta_mean:.3f} → 检查k2_gate/anti_cancel过度抑制"))
        if etheta_spike_pct > 0.10:
            issues.append(("⚠ 症状2 e_theta地板缝扰动",
                           f"spike_pct={etheta_spike_pct:.1%}, std={etheta_noise_std:.4f}rad → 降低e_theta_lowpass.alpha"))
        if underdamped_pct > 0.30:
            issues.append(("⚠ 症状2 结构性ζ不足",
                           f"ζ_mean={zeta_mean:.3f}, <0.5帧={underdamped_pct:.1%} → 需增大q2(K2)"))

    # ── 症状3：不良进入条件 ──
    if abs(ct0) > 3.5:
        issues.append(("⚠ 症状3 不良进入",
                       f"ct[0]={ct0:.2f}mm — 收紧对齐门槛或前路径级联影响"))

    # ── 症状4：积分饱和 ──
    if intg_saturation > 0.40:
        issues.append(("⚠ 症状4 积分饱和",
                       f"intg_sat>{integral_max_param*0.85:.4f}: {intg_saturation:.1%} → 增大integral_max或调整Ki"))

    # ── 正常 ──
    if not issues and p90 < 5.0:
        issues.append(("✓ 优秀", f"p90={p90:.2f}mm, ζ_mean={zeta_mean:.3f}"))
    elif not issues:
        issues.append(("△ 有改进空间", f"p90={p90:.2f}mm, ζ_mean={zeta_mean:.3f}"))

    return issues, row_diag


def layer2_paths(bm: pd.DataFrame, all_samples: dict):
    section("层2 · 路径逐一诊断（tuning_guide 决策树）")

    all_diags = {}
    for _, row in bm.iterrows():
        s       = int(row["path_slot"])
        samples = all_samples.get(s)
        if samples is None:
            continue

        p90   = row["p90_mm"]
        lt3   = row["ratio_lt_3mm"]
        pk    = "✓ 达标" if row["pass_p90_lt_5mm"] else "✗ 未达标"
        drift = row["drift_tail_minus_head_mm"]
        seg1  = row["seg1_ratio_lt_3mm"]
        seg3  = row["seg3_ratio_lt_3mm"]

        subsec(f"path_{s:02d}  {pk}  p90={p90:.2f}mm  <3mm%={lt3*100:.1f}%")

        ct       = samples["cross_track_mm"]
        ct_mean  = ct.mean(); ct_std = ct.std()
        ct0      = ct.iloc[0]
        v0       = samples["linear_speed_mps"].iloc[0]

        # K2 指标
        k2_mean   = samples["k2"].mean()          if "k2" in samples.columns else float("nan")
        floor_pct = samples["k2_floor_active"].mean() * 100 if "k2_floor_active" in samples.columns else 0
        cancel_m  = samples["cancel_ratio"].mean()       if "cancel_ratio" in samples.columns else 0
        cancel_h  = (samples["cancel_ratio"] > 0.5).mean() * 100 if "cancel_ratio" in samples.columns else 0

        # 角速度饱和
        if "omega_before_limits" in samples.columns and "phase_max_w" in samples.columns:
            sat_pct = (samples["omega_before_limits"].abs() > samples["phase_max_w"] * 0.95).mean() * 100
        else:
            sat_pct = 0

        # 阻尼比 ζ
        spd = samples["linear_speed_mps"].replace(0, np.nan)
        if "k1" in samples.columns:
            zeta_s = samples["k2"] / (2 * (spd * samples["k1"]).pow(0.5))
            zm = zeta_s.mean(); zw = zeta_s.min()
            under50 = (zeta_s < 0.5).mean() * 100
        else:
            zm = zw = float("nan"); under50 = 0

        # 积分
        intg_max  = samples["integral_state"].abs().max()  if "integral_state" in samples.columns else 0
        intg_mean = samples["integral_state"].mean()       if "integral_state" in samples.columns else 0
        intg_sat  = (samples["integral_state"].abs() > 0.0127).mean() * 100 \
                    if "integral_state" in samples.columns else 0  # 阈值 = 85% × integral_max=0.015

        # e_theta 噪声
        if "e_theta_raw_rad" in samples.columns:
            en     = (samples["e_theta_raw_rad"] - samples["e_theta_rad"]).abs()
            en_std = en.std(); en_spk = (en > 0.02).mean() * 100
        else:
            en_std = 0; en_spk = 0

        print(f"  进入: ct[0]={ct0:+.2f}mm  v[0]={v0:.3f}m/s  {'⚠ 不良进入!' if abs(ct0)>3.5 else '✓ 进入OK'}")
        print(f"  分布: mean={ct_mean:+.2f}mm  std={ct_std:.2f}mm  seg1<3mm={seg1*100:.1f}%  seg3<3mm={seg3*100:.1f}%  drift={drift:+.2f}mm")
        print(f"  K2  : k2_mean={k2_mean:.4f}  floor%={floor_pct:.1f}%  cancel_mean={cancel_m:.3f}  cancel>0.5={cancel_h:.1f}%")
        print(f"  ζ   : mean={zm:.3f}  worst={zw:.3f}  ζ<0.5帧={under50:.1f}%  {'⚠ 结构性不足!' if under50>20 else '✓'}")
        print(f"  饱和: w_sat={sat_pct:.1f}%  {'⚠ 角速度过饱和!' if sat_pct>8 else '✓'}")
        print(f"  积分: max={intg_max:.5f}  mean={intg_mean:.5f}  >85%上限={intg_sat:.1f}%  {'⚠ 积分饱和风险' if intg_sat>40 else '✓'}")
        print(f"  噪声: e_theta噪声std={en_std:.4f}rad  >0.02rad={en_spk:.1f}%  {'⚠ 地板缝干扰!' if en_spk>8 else '✓'}")

        # 诊断结论
        issues, row_diag = diagnose_path(s, row, samples)
        print(f"\n  诊断结论:")
        for tag, msg in issues:
            print(f"    {tag}: {msg}")

        all_diags[s] = (issues, row_diag)

    return all_diags

# ─────────────────── 层3：最差区段样本深析 ───────────────────
def layer3_worst(s: int, samples: pd.DataFrame, n_worst=30):
    subsec(f"层3 · path_{s:02d} 最差区段样本（{n_worst} 个）")

    ct      = samples["cross_track_mm"].abs()
    p90_val = ct.quantile(0.9)

    # 寻找最长连续违规区段（|ct|>4.5mm）
    breach    = samples["cross_track_mm"].abs() > 4.5
    max_run   = 0; cur_run = 0; best_start = 0; cur_start = 0
    for i, b in enumerate(breach):
        if b:
            if cur_run == 0: cur_start = i
            cur_run += 1
            if cur_run > max_run:
                max_run = cur_run; best_start = cur_start
        else:
            cur_run = 0

    # 区段前后各扩展 5 个样本
    ws  = max(0, best_start - 5)
    we  = min(len(samples), best_start + max_run + 5)
    seg = samples.iloc[ws:we]

    print(f"  最长 |ct|>4.5mm 连续区段: {max_run} 样本 (索引 {best_start}~{best_start+max_run})")
    print(f"  p90(|ct|) = {p90_val:.3f}mm\n")

    cols = ["cross_track_mm","e_y_filtered_mm","e_theta_rad","k1","k2","cancel_ratio",
            "integral_state","linear_speed_mps","angular_cmd_rps","k2_floor_active"]
    cols = [c for c in cols if c in samples.columns]

    fmt = "{:>5} " + " ".join(["{:>9}"] * len(cols))
    hdr = ["idx"] + [c[:9] for c in cols]
    print(fmt.format(*hdr))
    print("  " + "─" * (6 + 10 * len(cols)))
    for idx, row in seg.iterrows():
        vals = [idx] + [f"{row[c]:.4f}" if c != "k2_floor_active" else f"{int(row[c])}" for c in cols]
        print(fmt.format(*vals))

    # 区段摘要
    print(f"\n  区段摘要:")
    print(f"    ct_mean={seg['cross_track_mm'].mean():.2f}mm, ct_range=[{seg['cross_track_mm'].min():.2f}, {seg['cross_track_mm'].max():.2f}]")
    if "k2" in seg.columns:
        print(f"    k2_mean={seg['k2'].mean():.4f}, floor激活%={seg['k2_floor_active'].mean()*100:.1f}%")
    if "cancel_ratio" in seg.columns:
        print(f"    cancel_mean={seg['cancel_ratio'].mean():.3f}, >0.5={( seg['cancel_ratio']>0.5).mean()*100:.1f}%")
    if "integral_state" in seg.columns:
        print(f"    积分范围 [{seg['integral_state'].min():.5f}, {seg['integral_state'].max():.5f}]")

# ─────────────────── 优化建议 ───────────────────
def recommend(bm: pd.DataFrame, all_diags: dict, all_samples: dict):
    section("优化建议")

    # 读取当前 line.yaml 参数
    yaml_path = Path(__file__).parent.parent / "src/xline_follow_controller/config/line.yaml"
    current_params = {}
    if yaml_path.exists():
        import re
        txt = yaml_path.read_text()
        for key in ["q1","q2","K1_max","Ki","integral_max","k2_min_floor_scale",
                    "k2_gate.ey_end","k2_gate.min_scale","k2_anti_cancel.scale",
                    "line_cross_track_tolerance","e_theta_lowpass.alpha","ey_start"]:
            m = re.search(rf"^\s+{re.escape(key.split('.')[-1])}:\s+([\d.]+)", txt, re.MULTILINE)
            if m: current_params[key] = float(m.group(1))

    print(f"  当前参数 (line.yaml):")
    for k, v in current_params.items():
        print(f"    {k:<35} = {v}")

    print()

    # 全局统计
    passes    = int(bm["pass_p90_lt_5mm"].sum())
    avg_lt3   = bm["ratio_lt_3mm"].mean()
    b33_avg_lt3 = sum(B33[s]["lt3"] for s in B33) / 6
    # 最优历史基准
    best_batch  = max(BASELINES, key=lambda lb: sum(BASELINES[lb][s]["lt3"] for s in BASELINES[lb]) / len(BASELINES[lb]))
    best_avg_lt3 = sum(BASELINES[best_batch][s]["lt3"] for s in BASELINES[best_batch]) / len(BASELINES[best_batch])
    print(f"  当前状态: pass={passes}/6  avg<3mm%={avg_lt3*100:.1f}%  (历史最优={best_batch[:3]} {best_avg_lt3*100:.1f}%)")
    print()

    # 按症状分类汇总
    problems = {"不良进入":[], "末段漂移":[], "振荡":[], "积分饱和":[], "e_theta噪声":[]}
    recs = []

    for s, (issues, diag) in all_diags.items():
        for tag, msg in issues:
            if "进入" in tag:        problems["不良进入"].append(s)
            if "漂移" in tag:        problems["末段漂移"].append(s)
            if "振荡" in tag:        problems["振荡"].append(s)
            if "积分" in tag:        problems["积分饱和"].append(s)
            if "e_theta" in tag:     problems["e_theta噪声"].append(s)

    # ── 建议1：不良进入（cascade问题）──
    if problems["不良进入"]:
        cur_tol = current_params.get("line_cross_track_tolerance", 0.006)
        if cur_tol >= 0.006:
            recs.append({
                "优先级": 2,
                "症状": f"路径 {problems['不良进入']} 进入条件不良 (|ct[0]|>3.5mm)",
                "建议": f"line_cross_track_tolerance: {cur_tol:.3f} → {cur_tol-0.002:.3f}",
                "原因": "收紧对齐完成标准，降低跟随阶段初始偏差",
                "风险": "对齐阶段耗时略增"
            })
        else:
            recs.append({
                "优先级": 1,
                "症状": f"路径 {problems['不良进入']} 进入条件不良 — 对齐门槛已较紧({cur_tol*1000:.0f}mm)，疑似级联效应",
                "建议": "观察级联链路，分析前路径末段是否偏置",
                "原因": "门槛已收紧，再收紧效果有限",
                "风险": "对齐失败率上升"
            })

    # ── 建议2：ζ不足 → 增大q2 ──
    low_zeta_paths = [s for s, (_, d) in all_diags.items()
                      if d.get("underdamped_pct", 0) > 0.20]
    if low_zeta_paths:
        q2_cur = current_params.get("q2", 0.42)
        q2_new = round(q2_cur + 0.04, 2)
        K2_cur = math.sqrt(2 * math.sqrt(0.39 * q2_cur) + q2_cur)
        K2_new = math.sqrt(2 * math.sqrt(0.39 * q2_new) + q2_new)
        recs.append({
            "优先级": 2,
            "症状": f"路径 {low_zeta_paths} ζ<0.5帧比例偏高",
            "建议": f"q2: {q2_cur} → {q2_new}  (K2: {K2_cur:.4f}→{K2_new:.4f})",
            "原因": f"ζ @v=0.25: {K2_cur/1.581:.3f}→{K2_new/1.581:.3f}",
            "风险": "anti_cancel K2略增，注意floor/anti_cancel平衡"
        })

    # ── 建议3：floor覆盖anti_cancel检查 ──
    q2_c    = current_params.get("q2", 0.42)
    floor_c = current_params.get("k2_min_floor_scale", 0.45)
    K2_base = math.sqrt(2 * math.sqrt(0.39 * q2_c) + q2_c)
    anti_cancel_eff = 0.45 * K2_base
    floor_eff       = floor_c * K2_base
    if floor_eff > anti_cancel_eff + 0.01:
        recs.append({
            "优先级": 1,
            "症状": "floor覆盖anti_cancel，path_01/02类型回退风险",
            "建议": f"k2_min_floor_scale: {floor_c} → {round(anti_cancel_eff/K2_base, 2)}",
            "原因": f"floor={floor_eff:.3f} > anti_cancel_alone={anti_cancel_eff:.3f} (+{floor_eff-anti_cancel_eff:.3f} 多余对抗力)",
            "风险": "大误差区段K2最小值略降"
        })
    else:
        recs.append({
            "优先级": 0,
            "症状": "floor vs anti_cancel 检查",
            "建议": f"当前OK: floor={floor_eff:.3f} ≈ anti_cancel={anti_cancel_eff:.3f}",
            "原因": "", "风险": ""
        })

    # ── 建议4：e_theta噪声 ──
    if problems["e_theta噪声"]:
        alpha_cur = current_params.get("e_theta_lowpass.alpha", 0.82)
        recs.append({
            "优先级": 3,
            "症状": f"路径 {problems['e_theta噪声']} e_theta 高频噪声严重",
            "建议": f"e_theta_lowpass.alpha: {alpha_cur} → {round(alpha_cur-0.03, 2)}",
            "原因": "加强地板缝瞬态扰动抑制",
            "风险": "真实航向变化响应略微变慢"
        })

    # ── 建议5：积分饱和 ──
    if problems["积分饱和"]:
        ki_cur = current_params.get("Ki", 0.42)
        im_cur = current_params.get("integral_max", 0.015)
        recs.append({
            "优先级": 1,
            "症状": f"路径 {problems['积分饱和']} 积分饱和 >40%帧",
            "建议": f"Ki: {ki_cur} → {round(ki_cur-0.03, 2)}  或增大 integral_max",
            "原因": "防止积分Windup，恢复线性控制区",
            "风险": "侧坡修正力略降"
        })

    # 输出
    recs.sort(key=lambda x: x["优先级"], reverse=True)
    for i, r in enumerate(recs, 1):
        p   = r["优先级"]
        tag = "🔴 立即" if p >= 2 else ("🟡 检查" if p == 1 else "🟢 确认")
        print(f"  [{i}] {tag}  {r['症状']}")
        print(f"       建议: {r['建议']}")
        if r["原因"]: print(f"       原因: {r['原因']}")
        if r["风险"]: print(f"       风险: {r['风险']}")
        print()

    # ── 下一批次参数汇总 ──
    print("  ┌─ 下批次建议参数（确认后应用）─────────────────────────────────")
    print(f"  │  optimization.id  = opt_{pd.Timestamp.now().strftime('%Y%m%d')}_bNEXT_..._v1")
    for r in recs:
        if r["优先级"] >= 1 and r["建议"] and "→" in r["建议"]:
            print(f"  │  {r['建议']}")
    print("  └────────────────────────────────────────────────────────────────")

# ─────────────────── 历史对比 ───────────────────
def compare_history(bm: pd.DataFrame):
    section("全历史对比")
    labels   = list(BASELINES.keys())
    cur_label = f"当前_{bm['batch_id'].iloc[0]}"
    cur_data  = {int(r["path_slot"]): {"lt3": r["ratio_lt_3mm"], "lt5": r["ratio_lt_5mm"], "p90": r["p90_mm"]}
                 for _, r in bm.iterrows()}
    all_b = {**BASELINES, cur_label: cur_data}

    # <3mm% 对比表
    print(f"  {'路径':<5}", end="")
    for lb in [*labels, cur_label]:
        print(f" {'<3mm%':>8}", end="")
    print()
    print(f"  {'─'*5}", end="")
    for lb in [*labels, cur_label]:
        short = lb[:6]
        print(f" {short:>8}", end="")
    print()
    for s in range(1, 7):
        print(f"  {s:<5}", end="")
        for lb in [*labels, cur_label]:
            v = all_b[lb].get(s, {}).get("lt3", float("nan"))
            if math.isnan(v): print(f" {'─':>8}", end="")
            else: print(f" {v*100:>7.1f}%", end="")
        print()
    print()
    print(f"  {'均值':<5}", end="")
    for lb in [*labels, cur_label]:
        vals = [all_b[lb].get(s, {}).get("lt3", float("nan")) for s in range(1, 7)]
        vals = [v for v in vals if not math.isnan(v)]
        avg  = sum(vals) / len(vals) if vals else float("nan")
        if math.isnan(avg): print(f" {'─':>8}", end="")
        else: print(f" {avg*100:>7.1f}%", end="")
    print()

    # p90 对比表
    print()
    print(f"  {'路径':<5}", end="")
    for lb in [*labels, cur_label]:
        print(f" {'p90mm':>8}", end="")
    print()
    print(f"  {'─'*5}", end="")
    for lb in [*labels, cur_label]:
        print(f" {'─'*8}", end="")
    print()
    for s in range(1, 7):
        print(f"  {s:<5}", end="")
        for lb in [*labels, cur_label]:
            v  = all_b[lb].get(s, {}).get("p90", float("nan"))
            pk = "✓" if v < 5.0 else " "
            if math.isnan(v): print(f" {'─':>8}", end="")
            else: print(f" {v:>6.2f}{pk} ", end="")
        print()
    print()
    print(f"  {'均值':<5}", end="")
    for lb in [*labels, cur_label]:
        vals = [all_b[lb].get(s, {}).get("p90", float("nan")) for s in range(1, 7)]
        vals = [v for v in vals if not math.isnan(v)]
        avg  = sum(vals) / len(vals) if vals else float("nan")
        if math.isnan(avg): print(f" {'─':>8}", end="")
        else: print(f" {avg:>7.2f}mm", end="")
    print()

# ─────────────────── 主程序 ───────────────────
def main():
    parser = argparse.ArgumentParser(description="line_tracking 综合分析")
    parser.add_argument("--path", type=int, default=None,
                        help="指定路径槽深层样本分析 (1~6)")
    parser.add_argument("--compare", action="store_true",
                        help="输出全历史对比表")
    parser.add_argument("--all-deep", action="store_true",
                        help="对全部路径执行层3深层分析")
    args = parser.parse_args()

    # 加载数据
    bm_file = DATA_DIR / "batch_metrics.csv"
    if not bm_file.exists():
        print(f"[错误] {bm_file} 不存在")
        sys.exit(1)
    bm = pd.read_csv(bm_file)

    all_samples = {}
    for s in range(1, 7):
        f = DATA_DIR / f"path_0{s}_samples.csv"
        if f.exists():
            all_samples[s] = pd.read_csv(f)

    # ── 执行 ──
    if args.compare:
        compare_history(bm)
        return

    if args.path:
        s = args.path
        if s not in all_samples:
            print(f"[错误] path_0{s}_samples.csv 不存在")
            sys.exit(1)
        layer3_worst(s, all_samples[s], n_worst=40)
        return

    # 默认：全量分析
    path_results = layer1_batch(bm)
    all_diags    = layer2_paths(bm, all_samples)
    recommend(bm, all_diags, all_samples)

    # 自动对最差2条路径执行层3
    worst_paths = sorted(path_results, key=lambda s: path_results[s]["p90"], reverse=True)[:2]
    section(f"层3 · 最差2条路径自动深析 (path {worst_paths})")
    for s in worst_paths:
        if s in all_samples:
            layer3_worst(s, all_samples[s])

    if args.all_deep:
        for s in range(1, 7):
            if s in all_samples and s not in worst_paths:
                layer3_worst(s, all_samples[s])

    hr("═")
    print("  分析完成。确认建议后修改 line.yaml → 实测 → 再分析。")
    hr("═")

if __name__ == "__main__":
    main()
