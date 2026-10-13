#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Render the closed-loop velocity limit simulations written by the gtest scenarios.

For every <name>.meta.json in the input directory, writes <name>.png with velocity, acceleration
and jerk over time (plus velocity over distance for map and stop scenarios) and the list of checks.
Also writes overview.png and index.html summarizing every scenario.

Usage: plot_velocity_limit_simulation.py [DATA_DIR] [--output OUTPUT_DIR]
The VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR environment variable takes precedence over DATA_DIR.
"""

import argparse
import csv
import glob
import html
import json
import math
import os
import sys
import textwrap

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Reference palette (light surface); categorical slots are assigned in fixed order.
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
BOUND = "#8a8984"
WINDOW = "#f0efec"
EGO = "#2a78d6"  # slot 1
LIMIT = "#eb6834"  # slot 2
UPSTREAM = "#1baf7a"  # slot 3
REFERENCE = "#eda100"  # slot 4
IMPLIED = "#e87ba4"  # slot 5
PLAN_FIELD = "#008300"  # slot 6
GOOD = "#0ca30c"
CRITICAL = "#d03b3b"

PLAN_PERIOD = 2.0  # [s] between overlaid plans
CHECK_LINE_HEIGHT = 0.15  # [in]
REFERENCE_STYLES = {  # label prefix written by the harness -> (legend label, dash pattern)
    "reference:": ("fastest descent to the limit", (0, (4, 2))),
    "reference from visibility:": ("fastest descent from zone visibility", (0, (1, 1.5))),
    "latest braking:": ("latest feasible braking", (0, (4, 2))),
}


def read_csv(path):
    """Read a CSV file into a list of row dictionaries."""
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def to_float(value):
    """Convert a CSV cell to float; empty cells become NaN."""
    return float(value) if value not in ("", None) else math.nan


def column(rows, key):
    """Extract one numeric column from CSV rows."""
    return [to_float(row[key]) for row in rows]


def load(prefix):
    """Load the metadata and CSV data written for one scenario."""
    with open(prefix + ".meta.json") as stream:
        meta = json.load(stream)
    samples = read_csv(prefix + ".samples.csv")
    plans = read_csv(prefix + ".plans.csv")
    references = read_csv(prefix + ".reference.csv")
    return meta, samples, plans, references


def style_axis(ax, ylabel):
    """Apply the recessive grid and axis styling."""
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8, linestyle="-")
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)
    ax.set_ylabel(ylabel, color=TEXT_SECONDARY, fontsize=9)


def check_bound(meta, name):
    """Return the bound of a named check, if present."""
    for check in meta["checks"]:
        if check["name"] == name and check["bound"] is not None:
            return check["bound"]
    return None


def clip_with_markers(ax, x, y, low, high, color):
    """Set y limits and mark the samples beyond them on the panel edge."""
    ax.set_ylim(low, high)
    above = [(xi, high) for xi, yi in zip(x, y) if yi > high]
    below = [(xi, low) for xi, yi in zip(x, y) if yi < low]
    for points, marker in ((above, "^"), (below, "v")):
        if points:
            xs, ys = zip(*points)
            ax.plot(
                xs, ys, linestyle="none", marker=marker, markersize=5, color=color, clip_on=False
            )


def plot_plans(ax, plans, domain):
    """Overlay the input and output plans once per PLAN_PERIOD as faint lines."""
    by_cycle = {}
    for row in plans:
        cycle_time = float(row["cycle_time"])
        if abs(cycle_time / PLAN_PERIOD - round(cycle_time / PLAN_PERIOD)) > 1e-6:
            continue
        by_cycle.setdefault((row["kind"], row["cycle"]), []).append(row)
    labelled = set()
    for (kind, _), rows in by_cycle.items():
        label = f"{kind} plan (every {PLAN_PERIOD:g} s)"
        ax.plot(
            column(rows, domain),
            column(rows, "velocity"),
            color=UPSTREAM if kind == "input" else EGO,
            alpha=0.35 if kind == "input" else 0.3,
            linewidth=0.9,
            label=None if label in labelled else label,
        )
        labelled.add(label)


def shade_overspeed(ax, x, velocity, limit):
    """Shade where the executed velocity exceeds the limit."""
    over = [not math.isnan(lim) and v > lim + 0.1 for v, lim in zip(velocity, limit)]
    if any(over):
        upper = [v if o else math.nan for v, o in zip(velocity, over)]
        lower = [lim if o else math.nan for lim, o in zip(limit, over)]
        ax.fill_between(x, lower, upper, color=LIMIT, alpha=0.15, linewidth=0, label="above limit")


def legend_above(ax, ncol):
    """Place the legend in one row above the panel."""
    ax.legend(
        loc="lower left",
        bbox_to_anchor=(0.0, 1.0),
        fontsize=7.5,
        frameon=False,
        ncol=ncol,
        borderaxespad=0.2,
        handlelength=2.5,
    )


def draw_windows(ax, meta, domain):
    """Shade the steady windows of the given domain and draw their targets."""
    for index, window in enumerate(meta["steady_windows"]):
        if window["domain"] != domain:
            continue
        ax.axvspan(window["begin"], window["end"], color=WINDOW, zorder=0)
        ax.hlines(
            window["target"],
            window["begin"],
            window["end"],
            color=BOUND,
            linewidth=1.0,
            label="steady window target" if index == 0 else None,
        )


def velocity_panel(ax, meta, samples, plans, references, domain):
    """Plot velocity over time or arc length with limits, plans and references."""
    x_key = "time" if domain == "time" else "s"
    x = column(samples, x_key)
    velocity = column(samples, "velocity")
    limit = column(samples, "expected_limit")
    draw_windows(ax, meta, domain)
    plot_plans(ax, plans, x_key)
    ax.plot(x, limit, color=LIMIT, linewidth=2.0, label="limit at ego")
    shade_overspeed(ax, x, velocity, limit)
    ax.plot(x, velocity, color=EGO, linewidth=2.0, label="ego (executed)")
    curves = {}
    for row in references:
        if row["domain"] == domain:
            curves.setdefault(row["label"], []).append(row)
    labelled = set()
    for label, rows in curves.items():
        prefix = max((p for p in REFERENCE_STYLES if label.startswith(p)), key=len)
        legend, dashes = REFERENCE_STYLES[prefix]
        ax.plot(
            column(rows, "x"),
            column(rows, "velocity"),
            color=REFERENCE,
            linewidth=1.6,
            linestyle=dashes,
            label=None if legend in labelled else legend,
            zorder=5,
        )
        labelled.add(legend)
    if domain == "distance":
        stop = meta["upstream"].get("stop_s")
        if stop is not None:
            ax.axvline(stop, color=TEXT_SECONDARY, linewidth=1.0)
            ax.annotate(
                "stop line",
                (stop, 0),
                xytext=(4, 4),
                textcoords="offset points",
                color=TEXT_SECONDARY,
                fontsize=8,
            )
        finite = [xi for xi in x if not math.isnan(xi)]
        ax.set_xlim(min(finite), max(finite) + 5.0)
    else:
        ax.set_xlim(0.0, max(x))
    shown = velocity + column(samples, "upstream_velocity") + column(plans, "velocity")
    top = max(v for v in shown if not math.isnan(v))
    ax.set_ylim(-0.3, top * 1.12 + 0.5)
    if domain == "time":
        mark_resets(ax, meta)
    style_axis(ax, "velocity [m/s]")
    legend_above(ax, 7)


def mark_resets(ax, meta):
    """Mark the cycles where a fixed profile restarted from the measured ego state."""
    times = [reset["time"] for reset in meta.get("profile_resets", [])]
    if times:
        top = ax.get_ylim()[1]
        ax.plot(
            times,
            [top] * len(times),
            linestyle="none",
            marker="v",
            markersize=7,
            color=TEXT_SECONDARY,
            clip_on=False,
            label="fixed profile restart",
        )


def acceleration_panel(ax, meta, samples):
    """Plot the executed acceleration and the velocity derivative over time."""
    t = column(samples, "time")
    acceleration = column(samples, "acceleration")
    implied = column(samples, "implied_acceleration")
    if "plan_acceleration" in samples[0]:
        ax.plot(
            t,
            column(samples, "plan_acceleration"),
            color=PLAN_FIELD,
            linewidth=0.8,
            label="plan acceleration_mps2 field",
        )
    ax.plot(t, implied, color=IMPLIED, linewidth=1.0, label="velocity change over 0.1 s")
    ax.plot(t, acceleration, color=EGO, linewidth=2.0, label="ego acceleration (executed)")
    deceleration = check_bound(meta, "deceleration_within_bound") or 1.0
    acceleration_bound = check_bound(meta, "acceleration_within_bound") or 1.0
    ax.axhline(-deceleration, color=BOUND, linewidth=1.0, label="bounds")
    ax.axhline(acceleration_bound, color=BOUND, linewidth=1.0)
    clip_with_markers(
        ax, t, implied, -1.6 * deceleration - 0.2, 1.6 * acceleration_bound + 0.2, IMPLIED
    )
    style_axis(ax, "acceleration [m/s²]")
    legend_above(ax, 4)


def jerk_panel(ax, meta, samples):
    """Plot the executed jerk over time."""
    t = column(samples, "time")
    jerk = column(samples, "jerk")
    ax.plot(t, jerk, color=EGO, linewidth=1.2, label="ego jerk")
    bound = check_bound(meta, "jerk_within_bound")
    if bound is not None:
        ax.axhline(bound, color=BOUND, linewidth=1.0, label="bounds")
        ax.axhline(-bound, color=BOUND, linewidth=1.0)
        scale = 1.8 * bound
    else:
        scale = max(3.0, max(abs(j) for j in jerk if not math.isnan(j)) * 1.1)
    clip_with_markers(ax, t, jerk, -scale, scale, EGO)
    style_axis(ax, "jerk [m/s³]")
    ax.set_xlabel("time [s]", color=TEXT_SECONDARY, fontsize=9)
    legend_above(ax, 2)


def checks_text(fig, meta, top_inches, height):
    """Two lines per check: verdict, name, value and bound, then the detail."""
    y = top_inches
    for check in meta["checks"]:
        status = "\u2713 PASS" if check["passed"] else "\u2717 FAIL"
        value = "inf" if check["value"] is None else f"{check['value']:.3f}"
        bound = "-" if check["bound"] is None else f"{check['bound']:.3f}"
        fig.text(
            0.04,
            y / height,
            f"{status}  {check['name']}: {value} (bound {bound})",
            fontsize=7.5,
            family="monospace",
            color=GOOD if check["passed"] else CRITICAL,
            va="top",
        )
        fig.text(
            0.04,
            (y - CHECK_LINE_HEIGHT) / height,
            f"          {check['detail']}",
            fontsize=7.5,
            family="monospace",
            color=TEXT_SECONDARY,
            va="top",
        )
        y -= 2 * CHECK_LINE_HEIGHT


def render(prefix, output_dir):
    """Render one scenario to <name>.png."""
    meta, samples, plans, references = load(prefix)
    has_distance = bool(meta["zones"]) or meta["upstream"].get("stop_s") is not None
    panels = 4 if has_distance else 3
    check_height = 2 * CHECK_LINE_HEIGHT * len(meta["checks"]) + 0.3
    height = 2.7 * panels + 1.4 + check_height
    fig = plt.figure(figsize=(13, height), facecolor=SURFACE)
    top = 1.0 - 1.4 / height
    bottom = (check_height + 0.55) / height
    grid = fig.add_gridspec(panels, 1, top=top, bottom=bottom, left=0.06, right=0.98, hspace=0.45)
    ax_velocity = fig.add_subplot(grid[0])
    ax_acceleration = fig.add_subplot(grid[1], sharex=ax_velocity)
    ax_jerk = fig.add_subplot(grid[2], sharex=ax_velocity)
    velocity_panel(ax_velocity, meta, samples, plans, references, "time")
    acceleration_panel(ax_acceleration, meta, samples)
    jerk_panel(ax_jerk, meta, samples)
    if has_distance:
        ax_distance = fig.add_subplot(grid[3])
        velocity_panel(ax_distance, meta, samples, plans, references, "distance")
        ax_distance.set_xlabel("arc length [m]", color=TEXT_SECONDARY, fontsize=9)

    verdict = "PASS" if meta["passed"] else "FAIL"
    verdict_color = GOOD if meta["passed"] else CRITICAL
    fig.text(
        0.04,
        1.0 - 0.25 / height,
        f"{meta['name']}  —  {meta['title']}",
        fontsize=13,
        color=TEXT_PRIMARY,
        weight="bold",
        va="top",
    )
    fig.text(
        0.98,
        1.0 - 0.25 / height,
        ("✓ " if meta["passed"] else "✗ ") + verdict,
        fontsize=13,
        color=verdict_color,
        weight="bold",
        va="top",
        ha="right",
    )
    upstream = meta["upstream"]
    subtitle = (
        f"{textwrap.fill(meta['description'], 190)}\n"
        f"level: {meta['level']}   stages: {' -> '.join(meta['stages'])}   targets: "
        f"{meta['targets']}   upstream: {upstream['profile']} {upstream['cruise_velocity']:.2f} m/s"
        f", {upstream['points']} points   limits: {meta['max_acceleration']} / "
        f"-{meta['max_deceleration']} m/s², {meta['max_jerk']} m/s³   "
        f"cycle: {meta['cycle_period']} s   "
        f"fixed profile: {'on' if meta.get('fixed_profile') else 'off'}"
    )
    fig.text(0.04, 1.0 - 0.6 / height, subtitle, fontsize=8.5, color=TEXT_SECONDARY, va="top")
    checks_text(fig, meta, check_height, height)

    path = os.path.join(output_dir, meta["name"] + ".png")
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return meta, path


def write_overview(results, output_dir):
    """Write a contact sheet of every scenario plot."""
    columns = 4
    rows = math.ceil(len(results) / columns)
    fig, axes = plt.subplots(rows, columns, figsize=(5 * columns, 4.2 * rows), facecolor=SURFACE)
    for ax in axes.flat:
        ax.axis("off")
    for ax, (meta, path) in zip(axes.flat, results):
        ax.imshow(plt.imread(path))
        verdict = "✓ PASS" if meta["passed"] else "✗ FAIL"
        ax.set_title(
            f"{verdict}  {meta['name']}", fontsize=9, color=GOOD if meta["passed"] else CRITICAL
        )
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "overview.png"), dpi=90, facecolor=SURFACE)
    plt.close(fig)


def write_index(results, output_dir):
    """Write an HTML table linking every scenario plot."""
    rows = []
    for meta, path in results:
        failed = [c["name"] for c in meta["checks"] if not c["passed"]]
        verdict = "PASS" if meta["passed"] else "FAIL"
        rows.append(
            f"<tr><td class='{verdict.lower()}'>{verdict}</td>"
            f"<td><a href='{html.escape(os.path.basename(path))}'>"
            f"{html.escape(meta['name'])}</a></td>"
            f"<td>{html.escape(meta['title'])}</td><td>{html.escape(meta['targets'])}</td>"
            f"<td>{html.escape(', '.join(failed))}</td></tr>"
        )
    page = (
        "<!doctype html><meta charset='utf-8'><title>Velocity limit simulations</title>"
        "<style>body{font:14px sans-serif;margin:24px;background:#fcfcfb;color:#0b0b0b}"
        "td,th{padding:4px 10px;text-align:left;vertical-align:top;border-bottom:1px solid #e4e3df}"
        ".pass{color:#0ca30c}.fail{color:#d03b3b}</style>"
        "<h1>Velocity limit closed-loop simulations</h1>"
        "<p><a href='overview.png'>overview.png</a></p>"
        "<table><tr><th>Result</th><th>Scenario</th><th>Title</th><th>Targets</th>"
        "<th>Failed checks</th></tr>" + "".join(rows) + "</table>"
    )
    with open(os.path.join(output_dir, "index.html"), "w") as stream:
        stream.write(page)


def main():
    """Render every scenario found in the data directory."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("data_dir", nargs="?", default=None)
    parser.add_argument("--output", default=None, help="directory for the images (default: data)")
    args = parser.parse_args()
    data_dir = os.environ.get("VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR") or args.data_dir
    if not data_dir:
        parser.error("no data directory given")
    output_dir = args.output or data_dir
    os.makedirs(output_dir, exist_ok=True)

    metas = glob.glob(os.path.join(data_dir, "*.meta.json"))
    prefixes = sorted(path[: -len(".meta.json")] for path in metas)
    if not prefixes:
        print(f"No simulation data in {data_dir}", file=sys.stderr)
        return 1
    results = []
    for prefix in prefixes:
        meta, path = render(prefix, output_dir)
        results.append((meta, path))
        print(f"{'PASS' if meta['passed'] else 'FAIL'}  {path}")
    write_overview(results, output_dir)
    write_index(results, output_dir)
    print(f"Overview: {os.path.join(output_dir, 'index.html')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
