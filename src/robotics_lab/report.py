"""Text, CSV and HTML reports."""

from __future__ import annotations

import csv
from html import escape
from pathlib import Path


def text_table(headers: list[str], rows: list[list[object]]) -> str:
    str_rows = [[_cell(value) for value in row] for row in rows]
    widths = [len(header) for header in headers]
    for row in str_rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))

    def line(cells: list[str]) -> str:
        return "  ".join(cell.ljust(widths[i]) for i, cell in enumerate(cells))

    rule = "  ".join("-" * width for width in widths)
    return "\n".join([line(headers), rule, *[line(row) for row in str_rows]])


def _cell(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def write_csv(path: Path | str, headers: list[str], rows: list[list[object]]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(headers)
        for row in rows:
            writer.writerow([_cell(value) if isinstance(value, float) else value for value in row])


def write_html(path: Path | str, title: str, sections: list[tuple[str, list[str], list[list[object]]]]) -> None:
    parts = [
        "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>",
        f"<title>{escape(title)}</title>",
        "<style>",
        "body{background:#faf7f0;color:#1c1a17;font-family:Georgia,serif;margin:2rem auto;max-width:960px;}",
        "h1{font-weight:400;font-size:1.8rem;border-bottom:1px solid #1c1a17;padding-bottom:.4rem;}",
        "h2{font-weight:400;color:#8a3a24;font-size:1.15rem;}",
        "table{border-collapse:collapse;width:100%;background:#fff;margin-bottom:1.5rem;}",
        "th,td{border:1px solid #e4dccb;padding:.35rem .5rem;text-align:left;font-family:ui-sans-serif,system-ui,sans-serif;font-size:13px;}",
        "th{background:#f3efe6;}",
        "p.note{color:#6b665e;font-size:.9rem;}",
        "</style></head><body>",
        f"<h1>{escape(title)}</h1>",
        "<p class='note'>Synthetic data. Numbers are measurements on that data.</p>",
    ]
    for heading, headers, rows in sections:
        parts.append(f"<h2>{escape(heading)}</h2><table><thead><tr>")
        parts.extend(f"<th>{escape(header)}</th>" for header in headers)
        parts.append("</tr></thead><tbody>")
        for row in rows:
            parts.append("<tr>" + "".join(f"<td>{escape(_cell(value))}</td>" for value in row) + "</tr>")
        parts.append("</tbody></table>")
    parts.append("</body></html>")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(parts))
