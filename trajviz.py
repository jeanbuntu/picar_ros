#!/usr/bin/env python3
"""
trajviz.py -- 3D trajectory visualizer for PLY files written by tf_bridge.

Usage:
    python3 trajviz.py <file.ply> [file2.ply ...]

Outputs trajviz_out.html.
Two sliders: spline order k (2–10) and smoothing s.
Click legend entries to toggle series.
"""

import sys
import os
import numpy as np
from scipy.interpolate import splprep, splev
import plotly.graph_objects as go

OUT_FILE = 'trajviz_out.html'

K_MIN, K_MAX = 2, 10
K_DEFAULT    = 3
N_SPL        = 400   # points per spline curve

# Smoothing: s = S_MULT * n_points.  0 = exact interpolation.
S_MULTS  = [0, 1e-5, 1e-4, 5e-4, 0.001, 0.005, 0.01, 0.05, 0.1]
S_LABELS = ['0 — exact', 'barely', 'very light', 'light',
            'moderate', 'medium', 'heavy', 'very heavy', 'extreme']
S_DEFAULT_IDX = 0   # start at exact

RYG_SCALE = [[0.0, '#e84040'], [0.5, '#f5c400'], [1.0, '#3ecf3e']]


def read_ply(path: str) -> np.ndarray:
    with open(path) as f:
        lines = f.read().splitlines()
    try:
        hi = lines.index('end_header')
    except ValueError:
        raise ValueError(f"No end_header in {path}")
    data = []
    for line in lines[hi + 1:]:
        parts = line.strip().split()
        if len(parts) >= 3:
            data.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(data, dtype=np.float64)


def fit_spline(pts: np.ndarray, k: int, s: float) -> np.ndarray | None:
    mask = np.ones(len(pts), dtype=bool)
    mask[1:] = np.any(pts[1:] != pts[:-1], axis=1)
    pts = pts[mask]
    if len(pts) < k + 1:
        return None
    try:
        tck, _ = splprep([pts[:, 0], pts[:, 1], pts[:, 2]], k=k, s=s)
        x, y, z = splev(np.linspace(0.0, 1.0, N_SPL), tck)
        return np.column_stack([x, y, z])
    except Exception:
        return None


def main():
    files = sys.argv[1:]
    if not files:
        print("Usage: python3 trajviz.py <file.ply> [file2.ply ...]")
        sys.exit(1)

    datasets = []
    for path in files:
        pts = read_ply(path)
        if len(pts) == 0:
            print(f"  warning: {path} — no points, skipping")
            continue
        label = os.path.basename(path).replace('.ply', '')
        print(f"  {label}: {len(pts)} points")
        datasets.append((label, pts))

    if not datasets:
        print("No data.")
        sys.exit(1)

    n_files = len(datasets)
    k_vals  = list(range(K_MIN, K_MAX + 1))
    n_k     = len(k_vals)
    n_s     = len(S_MULTS)

    # ── Trace layout ──────────────────────────────────────────────────────────
    # [0 .. 3*n_files-1]            raw (line + start + end per file)
    # [3*n_files + (ki*n_s+si)*n_files + fi]   spline for (ki,si), file fi
    n_raw = 3 * n_files
    traces = []

    # ── Raw traces ────────────────────────────────────────────────────────────
    for label, pts in datasets:
        t = np.linspace(0.0, 1.0, len(pts))
        traces.append(go.Scatter3d(
            x=pts[:, 0], y=pts[:, 1], z=pts[:, 2],
            mode='lines+markers',
            line=dict(color=t, colorscale=RYG_SCALE, width=3),
            marker=dict(size=2.5, color=t, colorscale=RYG_SCALE, opacity=0.5),
            name=f'{label}  raw',
            legendgroup=label,
            opacity=0.55,
        ))
        traces.append(go.Scatter3d(
            x=[pts[0, 0]], y=[pts[0, 1]], z=[pts[0, 2]],
            mode='markers',
            marker=dict(size=7, color='#e84040', symbol='circle',
                        line=dict(color='white', width=1)),
            legendgroup=label, showlegend=False,
        ))
        traces.append(go.Scatter3d(
            x=[pts[-1, 0]], y=[pts[-1, 1]], z=[pts[-1, 2]],
            mode='markers',
            marker=dict(size=10, color='#3ecf3e', symbol='diamond',
                        line=dict(color='white', width=1)),
            legendgroup=label, showlegend=False,
        ))

    # ── Spline traces for every (k, s) combo ─────────────────────────────────
    print("  pre-computing splines...", end='', flush=True)
    for ki, k in enumerate(k_vals):
        for si, s_mult in enumerate(S_MULTS):
            for label, pts in datasets:
                s_val   = s_mult * len(pts)
                spl     = fit_spline(pts, k, s_val)
                is_default = (k == K_DEFAULT and si == S_DEFAULT_IDX)
                if spl is not None:
                    t = np.linspace(0.0, 1.0, len(spl))
                    traces.append(go.Scatter3d(
                        x=spl[:, 0], y=spl[:, 1], z=spl[:, 2],
                        mode='lines',
                        line=dict(color=t, colorscale=RYG_SCALE, width=5),
                        name=f'{label}  spline',
                        legendgroup=f'{label}_spl',
                        visible=is_default,
                        opacity=0.95,
                    ))
                else:
                    traces.append(go.Scatter3d(
                        x=[], y=[], z=[], mode='lines',
                        name=f'{label}  spline',
                        legendgroup=f'{label}_spl',
                        visible=is_default,
                    ))
    print(" done")

    fig = go.Figure(data=traces)
    fig.update_layout(
        paper_bgcolor='#1a1a1a',
        scene=dict(
            bgcolor='#1a1a1a',
            xaxis=dict(title='X (m)', color='#cccccc', gridcolor='#333333',
                       showbackground=False, zerolinecolor='#555555'),
            yaxis=dict(title='Y (m)', color='#cccccc', gridcolor='#333333',
                       showbackground=False, zerolinecolor='#555555'),
            zaxis=dict(title='Z (m)', color='#cccccc', gridcolor='#333333',
                       showbackground=False, zerolinecolor='#555555'),
            aspectmode='data',
        ),
        legend=dict(
            bgcolor='#222222', bordercolor='#555555', borderwidth=1,
            font=dict(color='#cccccc', size=11),
            itemclick='toggle', itemdoubleclick='toggleothers',
        ),
        margin=dict(l=0, r=0, t=10, b=0),
        title=None,
    )

    # ── Inject custom slider UI ───────────────────────────────────────────────
    slider_html = f"""
<div id="ctrl" style="
    position:fixed; bottom:24px; left:50%; transform:translateX(-50%);
    background:#1e1e1e; padding:16px 28px 14px; border-radius:10px;
    border:1px solid #444; z-index:9999; color:#ccc;
    font-family:'Segoe UI',sans-serif; font-size:13px;
    box-shadow:0 4px 18px rgba(0,0,0,0.6); min-width:360px;">
  <div style="margin-bottom:12px">
    <div style="display:flex; justify-content:space-between; margin-bottom:4px">
      <span>Spline order</span>
      <span style="color:#f5c400; font-weight:600">k = <span id="kv">{K_DEFAULT}</span></span>
    </div>
    <input type="range" id="ksl" min="{K_MIN}" max="{K_MAX}" value="{K_DEFAULT}" step="1"
           style="width:100%; accent-color:#f5c400" oninput="upd()">
    <div style="display:flex; justify-content:space-between; color:#666; font-size:11px; margin-top:2px">
      <span>{K_MIN}</span><span>{K_MAX}</span>
    </div>
  </div>
  <div>
    <div style="display:flex; justify-content:space-between; margin-bottom:4px">
      <span>Smoothing</span>
      <span style="color:#3ecf3e; font-weight:600" id="sv">{S_LABELS[S_DEFAULT_IDX]}</span>
    </div>
    <input type="range" id="ssl" min="0" max="{n_s - 1}" value="{S_DEFAULT_IDX}" step="1"
           style="width:100%; accent-color:#3ecf3e" oninput="upd()">
    <div style="display:flex; justify-content:space-between; color:#666; font-size:11px; margin-top:2px">
      <span>none</span><span>heavy</span>
    </div>
  </div>
</div>
<script>
const S_LABELS = {S_LABELS};
const N_FILES  = {n_files};
const N_K      = {n_k};
const N_S      = {n_s};
const N_RAW    = {n_raw};
const K_MIN    = {K_MIN};

function upd() {{
  const ki = parseInt(document.getElementById('ksl').value) - K_MIN;
  const si = parseInt(document.getElementById('ssl').value);
  document.getElementById('kv').textContent = ki + K_MIN;
  document.getElementById('sv').textContent = S_LABELS[si];

  const gd  = document.querySelector('.plotly-graph-div');
  const vis = [];
  for (let i = 0; i < N_RAW; i++) vis.push(true);
  for (let ki2 = 0; ki2 < N_K; ki2++)
    for (let si2 = 0; si2 < N_S; si2++)
      for (let fi = 0; fi < N_FILES; fi++)
        vis.push(ki2 === ki && si2 === si);

  Plotly.restyle(gd, {{'visible': vis}});
}}
</script>
"""

    html = fig.to_html(include_plotlyjs='cdn', full_html=True)
    html = html.replace('</body>', slider_html + '</body>')

    with open(OUT_FILE, 'w') as f:
        f.write(html)

    print(f"\nSaved → {os.path.abspath(OUT_FILE)}")
    print("Sliders: spline order k=2..10, smoothing s=none..heavy")
    print("Legend:  click to toggle, double-click to isolate")


if __name__ == '__main__':
    main()
