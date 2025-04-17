import pandas as pd, numpy as np, plotly.graph_objects as go
from scipy.spatial.transform import Rotation as R

CSV_FILE  = "trajectory_attitude_env.csv"   # ←パス合わせて
AXIS_LEN  = 0.5     # 矢印長さ
MAX_F     = 120     # フレーム数
LW        = 5       # 矢印線幅

# 1. データ読み込み
df   = pd.read_csv(CSV_FILE)
t    = df["Time(s)"].to_numpy()
pos  = df[["PosX","PosY","PosZ"]].to_numpy()
rot  = R.from_quat(df[["QuatX","QuatY","QuatZ","QuatW"]].to_numpy())
dirs_x, dirs_y, dirs_z = rot.apply([1,0,0]), rot.apply([0,1,0]), rot.apply([0,0,1])

# 2. Figure (単一 3D scene)
fig = go.Figure()

# 静的青い軌跡
fig.add_trace(go.Scatter3d(x=pos[:,0], y=pos[:,1], z=pos[:,2],
                           mode="lines", name="軌跡", line=dict(color="blue")))

# 初期の赤点＋姿勢矢印
def make_dyn_traces(idx):
    p, ex, ey, ez = pos[idx], dirs_x[idx], dirs_y[idx], dirs_z[idx]
    return [
        go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]],
                     mode="markers", marker=dict(color="red", size=4),
                     showlegend=False),
        go.Scatter3d(x=[p[0], p[0]+ex[0]*AXIS_LEN],
                     y=[p[1], p[1]+ex[1]*AXIS_LEN],
                     z=[p[2], p[2]+ex[2]*AXIS_LEN],
                     mode="lines", line=dict(color="green", width=LW),
                     showlegend=False),
        go.Scatter3d(x=[p[0], p[0]+ey[0]*AXIS_LEN],
                     y=[p[1], p[1]+ey[1]*AXIS_LEN],
                     z=[p[2], p[2]+ey[2]*AXIS_LEN],
                     mode="lines", line=dict(color="cyan", width=LW),
                     showlegend=False),
        go.Scatter3d(x=[p[0], p[0]+ez[0]*AXIS_LEN],
                     y=[p[1], p[1]+ez[1]*AXIS_LEN],
                     z=[p[2], p[2]+ez[2]*AXIS_LEN],
                     mode="lines", line=dict(color="red", width=LW),
                     showlegend=False)
    ]

fig.add_traces(make_dyn_traces(0))

# 3. フレーム生成
step = max(1, len(df)//MAX_F)
frames = []
for i in range(0, len(df), step):
    frames.append(go.Frame(data=[fig.data[0], *make_dyn_traces(i)],
                           name=f"{t[i]:.2f}"))
fig.frames = frames

# 4. レイアウト
fig.update_layout(
    scene=dict(aspectmode="data",
               xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)"),
    height=720,
    title="ロケット飛行シミュ",
    sliders=[dict(
        steps=[dict(method="animate",
                    args=[[f.name], {"frame":{"duration":0,"redraw":True},
                                     "mode":"immediate"}],
                    label=f.name) for f in frames],
        x=0, y=0, currentvalue=dict(prefix="Time(s): "), len=1.0)],
    updatemenus=[dict(type="buttons", showactive=False, x=1.05, y=0,
                      buttons=[dict(label="▶", method="animate",
                                    args=[None, {"frame":{"duration":50,"redraw":True},
                                                 "fromcurrent":True}])])]
)

fig.show()
