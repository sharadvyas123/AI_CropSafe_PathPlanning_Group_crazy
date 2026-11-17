"""SVG visualization for full-field watering coverage."""

from __future__ import annotations

from pathlib import Path

from .model import Field, Position

COLORS = {"watered": "#d9f0d3", "unwatered": "#f5f2e9", "protected": "#76a9d8", "obstacle": "#31343b"}


def save_coverage_plot(field: Field, watered: set[Position], output_path: Path) -> None:
    """Save one field map showing watered cells and excluded cells."""
    cell, left, top = 42, 64, 70
    width, height = left + field.width * cell + 20, top + field.height * cell + 110
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="34" text-anchor="middle" font-family="sans-serif" font-size="21" font-weight="bold">Full-field watering coverage</text>',
    ]
    for row in range(field.height):
        for col in range(field.width):
            pos = (row, col)
            color = COLORS["watered"] if pos in watered else COLORS["unwatered"]
            label = "W" if pos in watered else ""
            if pos in field.protected:
                color, label = COLORS["protected"], "P"
            elif pos in field.obstacles:
                color, label = COLORS["obstacle"], "X"
            x, y = left + col * cell, top + row * cell
            elements.append(f'<rect x="{x}" y="{y}" width="{cell}" height="{cell}" fill="{color}" stroke="#aaa" stroke-width="1"/>')
            if label:
                fill = "white" if label == "X" else "#244b22" if label == "W" else "#17324d"
                elements.append(f'<text x="{x+cell/2}" y="{y+cell/2+5}" text-anchor="middle" font-family="sans-serif" font-size="14" font-weight="bold" fill="{fill}">{label}</text>')
    legend = [("Watered cell", COLORS["watered"]), ("Protected, not watered", COLORS["protected"]), ("Obstacle, not watered", COLORS["obstacle"])]
    legend_y = top + field.height * cell + 34
    for i, (label, color) in enumerate(legend):
        x = left + i * (width - left) / len(legend)
        elements.append(f'<rect x="{x}" y="{legend_y-12}" width="14" height="14" fill="{color}" stroke="#777"/>')
        elements.append(f'<text x="{x+19}" y="{legend_y}" font-family="sans-serif" font-size="11">{label}</text>')
    elements.append(f'<text x="{width/2}" y="{legend_y+34}" text-anchor="middle" font-family="sans-serif" font-size="13">{len(watered)} of {len(field.irrigable)} irrigable cells watered</text>')
    elements.append('</svg>')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(elements), encoding="utf-8")
