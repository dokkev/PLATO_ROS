#!/usr/bin/env python3
"""Compare MPPI predicted rollout CSV rows against future measured tick rows.

Assumption: horizon_index = 1 corresponds to one controller tick in the future.
TODO: add a latency sweep over +/-3 ticks once we know the sensor/driver delay.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import defaultdict
from typing import Dict, Iterable, List, Tuple


def parse_vector(cell: str) -> List[float]:
    if cell is None:
        return []
    value = cell.strip()
    if not value:
        return []
    return [float(part) for part in value.split(";") if part != ""]


def squared_error(a: Iterable[float], b: Iterable[float]) -> Tuple[float, int]:
    total = 0.0
    count = 0
    for x, y in zip(a, b):
        if math.isfinite(x) and math.isfinite(y):
            diff = x - y
            total += diff * diff
            count += 1
    return total, count


def rmse(sum_sq: float, count: int) -> float:
    return math.sqrt(sum_sq / count) if count > 0 else float("nan")


def load_csv(path: str) -> List[dict]:
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def load_ticks(path: str) -> Dict[int, dict]:
    rows = {}
    for row in load_csv(path):
        rows[int(row["tick_index"])] = row
    return rows


def load_rollouts(path: str) -> List[dict]:
    return load_csv(path)


def fit_tau_scale(pairs_by_joint: Dict[int, List[Tuple[float, float]]]) -> Dict[int, Tuple[float, float]]:
    fits = {}
    for joint, pairs in pairs_by_joint.items():
        pairs = [(x, y) for x, y in pairs if math.isfinite(x) and math.isfinite(y)]
        if len(pairs) < 2:
            fits[joint] = (float("nan"), float("nan"))
            continue
        mean_x = sum(x for x, _ in pairs) / len(pairs)
        mean_y = sum(y for _, y in pairs) / len(pairs)
        var_x = sum((x - mean_x) ** 2 for x, _ in pairs)
        if var_x <= 0.0:
            fits[joint] = (float("nan"), mean_y)
            continue
        cov_xy = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
        alpha = cov_xy / var_x
        beta = mean_y - alpha * mean_x
        fits[joint] = (alpha, beta)
    return fits


def maybe_plot(output_dir: str, horizon_rows: List[dict]) -> None:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return

    horizons = [int(row["horizon_index"]) for row in horizon_rows]
    for key, title in (
        ("q_rmse", "q RMSE by horizon"),
        ("qdot_rmse", "qdot RMSE by horizon"),
        ("tau_rmse", "tau RMSE by horizon"),
    ):
        values = [float(row[key]) for row in horizon_rows]
        plt.figure()
        plt.plot(horizons, values, marker="o")
        plt.xlabel("horizon_index")
        plt.ylabel(key)
        plt.title(title)
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"{key}_by_horizon.png"))
        plt.close()


def evaluate(ticks_path: str, rollouts_path: str, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)
    ticks = load_ticks(ticks_path)
    rollouts = load_rollouts(rollouts_path)

    sums = defaultdict(lambda: {
        "samples": 0,
        "q_sum_sq": 0.0,
        "q_count": 0,
        "qdot_sum_sq": 0.0,
        "qdot_count": 0,
        "tau_sum_sq": 0.0,
        "tau_count": 0,
    })
    tau_pairs_by_joint: Dict[int, List[Tuple[float, float]]] = defaultdict(list)
    matched_predictions = 0
    skipped_predictions = 0

    for pred in rollouts:
        tick_index = int(pred["tick_index"])
        horizon_index = int(pred["horizon_index"])
        future_tick = tick_index + horizon_index
        measured = ticks.get(future_tick)
        if measured is None:
            skipped_predictions += 1
            continue

        q_sum, q_count = squared_error(
            parse_vector(pred.get("q_pred", "")),
            parse_vector(measured.get("q_meas", "")),
        )
        qdot_sum, qdot_count = squared_error(
            parse_vector(pred.get("qdot_pred", "")),
            parse_vector(measured.get("qdot_meas", "")),
        )
        tau_pred = parse_vector(pred.get("tau_pred", ""))
        tau_meas = parse_vector(measured.get("tau_meas", ""))
        tau_sum, tau_count = squared_error(tau_pred, tau_meas)

        bucket = sums[horizon_index]
        bucket["samples"] += 1
        bucket["q_sum_sq"] += q_sum
        bucket["q_count"] += q_count
        bucket["qdot_sum_sq"] += qdot_sum
        bucket["qdot_count"] += qdot_count
        bucket["tau_sum_sq"] += tau_sum
        bucket["tau_count"] += tau_count
        matched_predictions += 1

        if horizon_index == 1:
            for joint, (pred_tau, meas_tau) in enumerate(zip(tau_pred, tau_meas)):
                tau_pairs_by_joint[joint].append((pred_tau, meas_tau))

    horizon_rows = []
    for horizon_index in sorted(sums):
        bucket = sums[horizon_index]
        horizon_rows.append({
            "horizon_index": horizon_index,
            "samples": bucket["samples"],
            "q_rmse": rmse(bucket["q_sum_sq"], bucket["q_count"]),
            "qdot_rmse": rmse(bucket["qdot_sum_sq"], bucket["qdot_count"]),
            "tau_rmse": rmse(bucket["tau_sum_sq"], bucket["tau_count"]),
        })

    with open(os.path.join(output_dir, "horizon_errors.csv"), "w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["horizon_index", "samples", "q_rmse", "qdot_rmse", "tau_rmse"],
        )
        writer.writeheader()
        writer.writerows(horizon_rows)

    tau_fits = fit_tau_scale(tau_pairs_by_joint)
    with open(os.path.join(output_dir, "summary.txt"), "w") as handle:
        handle.write("MPPI rollout log evaluation\n")
        handle.write(f"ticks: {ticks_path}\n")
        handle.write(f"rollouts: {rollouts_path}\n")
        handle.write(f"measured_tick_rows: {len(ticks)}\n")
        handle.write(f"rollout_rows: {len(rollouts)}\n")
        handle.write(f"matched_predictions: {matched_predictions}\n")
        handle.write(f"skipped_predictions_no_future_tick: {skipped_predictions}\n")
        handle.write("\nrmse_by_horizon:\n")
        for row in horizon_rows:
            handle.write(
                f"  h={row['horizon_index']}: samples={row['samples']} "
                f"q={row['q_rmse']:.8g} qdot={row['qdot_rmse']:.8g} "
                f"tau={row['tau_rmse']:.8g}\n"
            )
        handle.write("\none_step_tau_fit: tau_meas ~= alpha * tau_pred + beta\n")
        for joint in sorted(tau_fits):
            alpha, beta = tau_fits[joint]
            handle.write(f"  joint_{joint}: alpha={alpha:.8g} beta={beta:.8g}\n")

    maybe_plot(output_dir, horizon_rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ticks", required=True, help="Path to *_ticks.csv")
    parser.add_argument("--rollouts", required=True, help="Path to *_rollouts.csv")
    parser.add_argument("--output", required=True, help="Output summary directory")
    args = parser.parse_args()
    evaluate(args.ticks, args.rollouts, args.output)


if __name__ == "__main__":
    main()
