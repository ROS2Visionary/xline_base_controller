# LQR 直线跟随调优工具集

本目录包含 LQR 直线跟随控制器的全套调优文档与分析脚本。

---

## 文件说明

| 文件 | 说明 |
|------|------|
| `analyze.py` | **主分析脚本**：三层漏斗法自动诊断，含历史对比、建议输出 |
| `tuning_guide.md` | 调优指南：控制链路、诊断决策树、参数手册、历史教训 |
| `line_optimization_history.md` | 历次调优批次记录（B33→B39+） |

---

## 快速开始

### 分析最新批次（最常用）

```bash
# 从项目根目录运行
python3 lqr_tuning/analyze.py
```

输出三层报告：
1. **层1** — 批次概览：6路径 pass/fail、avg<3mm%、p90，对比 B33 基准和上批次
2. **层2** — 路径逐一诊断：进入条件、K2状态、ζ、积分饱和、5症状决策树
3. **层3** — 最差2条路径的30样本深析：帧级数据表 + 区段摘要

### 其他常用选项

```bash
# 单路径样本深层分析（如指定 path_02）
python3 lqr_tuning/analyze.py --path 02

# 全历史批次对比表
python3 lqr_tuning/analyze.py --compare

# 全部6条路径三层深析（输出较多）
python3 lqr_tuning/analyze.py --all-deep
```

---

## 标准调优工作流

```
1. 跑机采集数据  →  line_tracking_latest/ 自动更新
       ↓
2. python3 lqr_tuning/analyze.py  →  查看层1/层2/层3报告
       ↓
3. 按脚本"优化建议"确认改动（每次只改一个参数）
       ↓
4. 修改 src/xline_follow_controller/config/line.yaml
   - 更新 optimization.id（格式：opt_YYYYMMDD_bNN_描述_v1）
   - 更新 parent_batch_id（填上一批 batch_id）
   - 更新 change_note（说明改动内容和原因）
       ↓
5. 重新编译部署  →  回到步骤1
       ↓
6. 若新批次优于历史最优：
   cp config/line.yaml config/line_best.yaml
   （同步更新 line_optimization_history.md 的"当前最优"段落）
```

---

## 数据位置

| 路径 | 说明 |
|------|------|
| `line_tracking_latest/` | 最新批次采样数据（每次跑机后自动覆盖） |
| `src/xline_follow_controller/config/line.yaml` | 当前运行配置 |
| `src/xline_follow_controller/config/line_best.yaml` | 历史最优配置（手动备份） |
| `src/xline_follow_controller/config/line_second_best.yaml` | 次优配置 |

---

## 关键指标速查

| 指标 | 目标 | 告警阈值 |
|------|------|---------|
| `p90_mm`（6路径均值） | < 5mm | > 7mm |
| `<3mm%`（6路径均值） | > 65% | < 50% |
| pass 路径数 | 6/6 | < 4/6 |
| ζ（阻尼比均值） | 0.65~0.85 | < 0.5 |
| `cancel>0.5` 占比 | < 20% | > 40% |
| 积分饱和（>85%上限） | < 30% | > 70% |
| `w_saturation_ratio` | < 5% | > 10% |

---

## 历史基准参考

| 批次 | avg<3mm% | pass | 备注 |
|------|---------|------|------|
| B33 | 59.2% | 2/6 | 结构性修复后基准 |
| B35 | **62.1%** | **3/6** | **历史最优** |
| B36 | 60.5% | 3/6 | q2=0.42（K2提升） |
| B37 | 52.7% | 2/6 | align_tol=4mm+ey_start=3mm 双改回归 |
| B38 | 50.1% | 2/6 | align_tol=4mm 仍在，最差 |
| B39 | — | — | 恢复 align_tol=6mm，待测 |

---

## 调优原则（最重要的三条）

1. **每次只改一个参数**。同时改多个参数时无法判断哪个带来了改变。
2. **改参数前先计算 ζ**。`ζ = K2_eff / (2×√(v×K1))`，目标 ζ ∈ [0.65, 0.85]。
3. **floor = anti_cancel 不变量**：`k2_min_floor_scale` 必须等于 `k2_anti_cancel.scale`（当前均为 0.45），否则 floor 会覆盖 anti_cancel 保护机制。

详细规则见 `tuning_guide.md` 第6节"正确的实验设计"和第7节"历史错误模式与教训"。
