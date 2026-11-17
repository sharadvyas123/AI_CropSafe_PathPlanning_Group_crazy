"""Run all four full-field watering coverage planners."""

from __future__ import annotations

import csv
from pathlib import Path

from .coverage import evaluate_coverage
from .field_scenario import make_field
from .visualize import save_coverage_plot

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results"


def _ascii_coverage(field, watered: set[tuple[int, int]]) -> str:
    lines = ["Legend: W = watered, P = protected (excluded), X = obstacle (excluded)"]
    for row in range(field.height):
        cells = []
        for col in range(field.width):
            pos = (row, col)
            if pos in field.protected:
                cells.append("P")
            elif pos in field.obstacles:
                cells.append("X")
            elif pos in watered:
                cells.append("W")
            else:
                cells.append(".")
        lines.append(" ".join(cells))
    return "\n".join(lines)


def main() -> None:
    field = make_field()
    plans = evaluate_coverage(field)
    RESULTS.mkdir(exist_ok=True)
    with (RESULTS / "watering_coverage.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["Algorithm", "Irrigable cells", "Watered cells", "Coverage (%)", "Movement steps", "Expanded cells", "Time (ms)", "Complete"])
        for plan in plans:
            writer.writerow([plan.algorithm, len(plan.irrigable_cells), len(plan.watered_cells), f"{plan.coverage_percent:.2f}", plan.movement_steps, plan.expanded_nodes, f"{plan.elapsed_seconds * 1000:.3f}", plan.complete])

    print("Full-field watering coverage (no start or goal input)")
    print(f"{'Algorithm':<12} {'Watered':>9} {'Coverage':>10} {'Moves':>8} {'Expanded':>10} {'Time ms':>10} {'Complete':>10}")
    for plan in plans:
        print(f"{plan.algorithm:<12} {len(plan.watered_cells):>9}/{len(plan.irrigable_cells):<3} {plan.coverage_percent:>9.2f}% {plan.movement_steps:>8} {plan.expanded_nodes:>10} {plan.elapsed_seconds * 1000:>10.3f} {str(plan.complete):>10}")

    # Every planner must cover the exact same eligible area. One map is enough
    # to show that watering mask; route choice is summarized in the CSV.
    canonical = plans[-1]
    print("\nWatering map for the complete field:")
    print(_ascii_coverage(field, canonical.watered_cells))
    save_coverage_plot(field, canonical.watered_cells, RESULTS / "screenshots" / "watering_coverage.svg")
    print(f"\nSaved coverage report and map under {RESULTS.relative_to(ROOT)}/")


if __name__ == "__main__":
    main()
