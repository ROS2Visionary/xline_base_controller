# 曲线 LQR 调优历史记录

> 每次完成一轮测试后更新本文件。格式参考圆弧控制器的 circle_optimization_history.md。

---

## 批次记录模板

```markdown
### C00 — 初始基线 (batch_id: <待填>)

**变更**: 初始参数，未调优
**父批次**: 无

**关键配置**:
| 参数 | 值 |
|------|-----|
| feedback_limit_ratio | 0.10 |
| enable_integral | false |
| v_max | 0.05 m/s |
| savgol_window | 7 |
| q1/q2/r | 200/100/1.0 |

**测试结果**:
| 曲线 | p90(mm) | <3mm% | 偏向(mm) | 通过 |
|------|---------|-------|---------|------|
| curve_00 | — | — | — | — |

**avg_p90**: —  **avg<3mm%**: —  **通过率**: —/—

**诊断**:
- 待测

**结论**:
- 待测

---
```

---

## C00 — 架构修正基线（待测）

**变更**: 移植圆弧控制器算法 + 架构修正（K1/K2、SG滤波、反馈限制）
**父批次**: 无
**optimization_id**: curve_C00_arch_corrected

**架构变更说明**（一次性修正，非调优步骤）:

| 项目 | 原始值 | 修正值 | 原因 |
|------|--------|--------|------|
| use_direct_gains | false | true | 计算值K1=282.8→tanh退化bang-bang |
| K1_direct | (计算得282.8) | 8.0 | 参考圆弧C19=6.0，补偿曲线低速 |
| K2_direct | (计算得19.5) | 1.5 | 圆弧C19验证临界阻尼值 |
| SG滤波器 | Hampel+SG | 仅Hampel | SG引入~200ms=10mm位置滞后 |
| limit_ratio_before_print | 0.10 | 0.20 | K1=8时需>0.15才工作在比例区 |
| limit_ratio_after_print | 0.10 | 0.20 | 同上，待C01+分段调优 |

**移植自圆弧C19的算法**:
- e_theta IIR低通滤波（α=0.82）
- e_y跳变检测（5mm/周期冻结积分）
- tanh软饱和（比例路径）
- 双路径独立限幅（比例+积分分离）
- 时变积分衰减 decay^(ctrl_dt/T)
- 实际ctrl_dt测量
- 50ms精确采样间隔
- before/after_print平滑切换（τ=0.3s）

**关键配置（修正后）**:
| 参数 | 值 |
|------|-----|
| K1/K2（直接） | 8.0 / 1.5 |
| limit_ratio_before/after | 0.20 / 0.20 |
| feedback_min_limit | 0.0 |
| enable_integral | false |
| e_theta_lowpass_alpha | 0.82 |
| max_ey_jump_m | 0.005 |
| int_limit_ratio | 0.35 |
| v_max | 0.05 m/s |
| omega_max | 0.3 rad/s |
| lookahead | 0.01m + 0.2s = 20mm总前瞻 |
| 位置滤波 | 仅Hampel(w=5, k=3.0) |

**测试结果**: 待测

**batch_id**: —

---
