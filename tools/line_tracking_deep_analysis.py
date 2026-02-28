#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path
from statistics import mean


def percentile(values, p):
    if not values:
        return 0.0
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    rank = (p / 100.0) * (len(xs) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    t = rank - lo
    return xs[lo] + (xs[hi] - xs[lo]) * t


def longest_streak(mask):
    best_len = 0
    best_start = -1
    cur_start = -1
    cur_len = 0
    for i, flag in enumerate(mask):
        if flag:
            if cur_len == 0:
                cur_start = i
            cur_len += 1
            if cur_len > best_len:
                best_len = cur_len
                best_start = cur_start
        else:
            cur_len = 0
    return best_len, best_start


def stable_breach_count(values, thr, min_run):
    n = len(values)
    i = 0
    total = 0
    while i < n:
        if values[i] <= thr:
            i += 1
            continue
        j = i
        while j < n and values[j] > thr:
            j += 1
        if (j - i) >= min_run:
            total += (j - i)
        i = j
    return total


def to_float(row, key, default=float("nan")):
    v = row.get(key)
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def load_batch_metrics(batch_file):
    rows = []
    if not batch_file.exists():
        return rows
    with batch_file.open() as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def analyze_path(sample_file, follow_max_w):
    with sample_file.open() as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None

    t = [to_float(r, "t_s", 0.0) for r in rows]
    e = [abs(to_float(r, "cross_track_mm", 0.0)) for r in rows]
    over5 = [v > 5.0 for v in e]
    over10 = [v > 10.0 for v in e]
    under3_ratio = sum(1 for v in e if v < 3.0) / len(e)
    under5_ratio = sum(1 for v in e if v < 5.0) / len(e)
    stable_under3_ratio = 1.0 - stable_breach_count(e, 3.0, 3) / len(e)
    stable_under5_ratio = 1.0 - stable_breach_count(e, 5.0, 3) / len(e)

    l5, s5 = longest_streak(over5)
    l10, s10 = longest_streak(over10)

    canc = 0
    severe = 0
    sat = 0
    start_aligned_sum = 0.0
    start_aligned_count = 0

    for r in rows:
        ey_mm = to_float(r, "e_y_filtered_mm")
        et = to_float(r, "e_theta_rad")
        k1 = to_float(r, "k1")
        k2 = to_float(r, "k2")
        if math.isfinite(ey_mm) and math.isfinite(et) and math.isfinite(k1) and math.isfinite(k2):
            ey = ey_mm / 1000.0
            t1 = abs(k1 * ey)
            t2 = abs(k2 * et)
            if ey * et < 0.0 and t1 > 1e-3 and t2 > 1e-3:
                canc += 1
                if min(t1, t2) / max(t1, t2) > 0.6:
                    severe += 1

        phase_max_w = to_float(r, "phase_max_w", follow_max_w)
        w = abs(to_float(r, "angular_cmd_rps", 0.0))
        if math.isfinite(phase_max_w) and phase_max_w > 1e-6 and w >= 0.98 * phase_max_w:
            sat += 1

        sa = to_float(r, "start_aligned")
        if math.isfinite(sa):
            start_aligned_sum += sa
            start_aligned_count += 1

    def time_span(start_idx, length):
        if start_idx < 0 or length <= 0:
            return (0.0, 0.0)
        end_idx = min(len(t) - 1, start_idx + length - 1)
        return (t[start_idx], t[end_idx])

    t5s, t5e = time_span(s5, l5)
    t10s, t10e = time_span(s10, l10)

    n = len(rows)
    tail = e[2 * n // 3 :]
    tail_under3_ratio = sum(1 for v in tail if v < 3.0) / len(tail)
    tail_under5_ratio = sum(1 for v in tail if v < 5.0) / len(tail)
    tail_stable_under5_ratio = 1.0 - stable_breach_count(tail, 5.0, 3) / len(tail)
    objective_score = 0.7 * stable_under3_ratio + 0.3 * stable_under5_ratio

    return {
        "path": sample_file.stem.replace("_samples", ""),
        "samples": n,
        "p50_mm": percentile(e, 50.0),
        "p90_mm": percentile(e, 90.0),
        "p95_mm": percentile(e, 95.0),
        "max_mm": max(e),
        "mean_mm": mean(e),
        "under3_ratio": under3_ratio,
        "under5_ratio": under5_ratio,
        "stable_under3_ratio": stable_under3_ratio,
        "stable_under5_ratio": stable_under5_ratio,
        "over5_ratio": sum(over5) / n,
        "over10_ratio": sum(over10) / n,
        "longest_over5_samples": l5,
        "longest_over5_t_start": t5s,
        "longest_over5_t_end": t5e,
        "longest_over10_samples": l10,
        "longest_over10_t_start": t10s,
        "longest_over10_t_end": t10e,
        "cancel_ratio": canc / n,
        "severe_cancel_ratio": severe / n,
        "w_saturation_ratio": sat / n,
        "start_aligned_ratio": (start_aligned_sum / start_aligned_count) if start_aligned_count else float("nan"),
        "tail_under3_ratio": tail_under3_ratio,
        "tail_under5_ratio": tail_under5_ratio,
        "tail_stable_under5_ratio": tail_stable_under5_ratio,
        "objective_score": objective_score,
    }


def write_outputs(out_dir, batch_rows, analyzed_rows):
    by_path_file = out_dir / "deep_analysis_by_path.csv"
    fields = [
        "path", "samples", "p50_mm", "p90_mm", "p95_mm", "max_mm", "mean_mm",
        "under3_ratio", "under5_ratio", "stable_under3_ratio", "stable_under5_ratio",
        "over5_ratio", "over10_ratio", "longest_over5_samples", "longest_over5_t_start",
        "longest_over5_t_end", "longest_over10_samples", "longest_over10_t_start",
        "longest_over10_t_end", "cancel_ratio", "severe_cancel_ratio",
        "w_saturation_ratio", "start_aligned_ratio", "tail_under3_ratio",
        "tail_under5_ratio", "tail_stable_under5_ratio", "objective_score",
    ]
    with by_path_file.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in analyzed_rows:
            w.writerow(r)

    summary_file = out_dir / "deep_analysis_summary.md"
    pass_count = 0
    total = 0
    batch_id = ""
    if batch_rows:
        batch_id = batch_rows[0].get("batch_id", "")
        for r in batch_rows:
            total += 1
            pass_count += int(float(r.get("pass_p90_lt_5mm", "0")))

    worst_p90 = sorted(analyzed_rows, key=lambda x: x["p90_mm"], reverse=True)
    priority = sorted(analyzed_rows, key=lambda x: (x["stable_under3_ratio"], x["stable_under5_ratio"]))
    avg_stable_under3 = sum(r["stable_under3_ratio"] for r in analyzed_rows) / len(analyzed_rows)
    avg_stable_under5 = sum(r["stable_under5_ratio"] for r in analyzed_rows) / len(analyzed_rows)
    with summary_file.open("w") as f:
        f.write("# Deep Analysis Summary\n\n")
        if batch_id:
            f.write(f"- Batch ID: `{batch_id}`\n")
        if total:
            f.write(f"- Pass (<5mm P90): `{pass_count}/{total}`\n")
        f.write(f"- Avg Stable <3mm: `{avg_stable_under3*100:.1f}%`\n")
        f.write(f"- Avg Stable <5mm: `{avg_stable_under5*100:.1f}%`\n")
        f.write("\n## Priority Paths (optimize <3 then <5)\n")
        for r in priority[:3]:
            f.write(
                f"- {r['path']}: stable<3={r['stable_under3_ratio']*100:.1f}%, "
                f"stable<5={r['stable_under5_ratio']*100:.1f}%, "
                f"tail_stable<5={r['tail_stable_under5_ratio']*100:.1f}%\n"
            )
        f.write("\n## Worst Paths (by P90)\n")
        for r in worst_p90[:3]:
            f.write(
                f"- {r['path']}: p90={r['p90_mm']:.3f}mm, over5={r['over5_ratio']*100:.1f}%, "
                f"cancel={r['cancel_ratio']*100:.1f}%, sat={r['w_saturation_ratio']*100:.1f}%\n"
            )


def main():
    parser = argparse.ArgumentParser(description="Analyze line_tracking_latest samples.")
    parser.add_argument(
        "--dir",
        default="line_tracking_latest",
        help="Tracking directory containing batch_metrics.csv and path_*_samples.csv",
    )
    args = parser.parse_args()

    out_dir = Path(args.dir)
    batch_file = out_dir / "batch_metrics.csv"
    batch_rows = load_batch_metrics(batch_file)
    follow_max_by_slot = {}
    for r in batch_rows:
        try:
            slot = int(float(r.get("path_slot", "0")))
            follow_max_by_slot[slot] = float(r.get("follow_max_w", "nan"))
        except ValueError:
            continue

    analyzed_rows = []
    for sample_file in sorted(out_dir.glob("path_*_samples.csv")):
        slot_text = sample_file.stem.split("_")[1]
        slot = int(slot_text) if slot_text.isdigit() else 0
        follow_max_w = follow_max_by_slot.get(slot, float("nan"))
        r = analyze_path(sample_file, follow_max_w)
        if r:
            analyzed_rows.append(r)

    if not analyzed_rows:
        print("No sample files found or all empty.")
        return 1

    write_outputs(out_dir, batch_rows, analyzed_rows)

    print("Deep analysis written:")
    print(f"  - {out_dir / 'deep_analysis_by_path.csv'}")
    print(f"  - {out_dir / 'deep_analysis_summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
