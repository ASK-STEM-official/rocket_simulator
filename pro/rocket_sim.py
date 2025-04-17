# coding: utf-8
"""
rocket_sim_6dof_gust.py
───────────────────────
* 6 DoF ロケットシミュレーション (並進3 + 回転3)
* Lift / Side‑force / Pitch & Yaw モーメント (簡易線形係数)
* Dryden ガストモデルで乱流風を付加
  → パラメータは config.json に追加:
      "turbulence": {
          "sigma": [1.0, 1.0, 0.5],   # σu, σv, σw [m/s]
          "length": [200.0, 200.0, 50.0]  # Lu, Lv, Lw [m]
      }
"""
import json, csv, math, random
import numpy as np, trimesh
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

# ────────────────────────────────────── ユーティリティ
normalize = lambda v: v/np.linalg.norm(v) if np.linalg.norm(v)!=0 else v
rho_atm = lambda z,p: p["rho0"]*math.exp(-z/p["scale_height"]) if p["type"]=="exponential" else p.get("rho0",1.225)

# 風 + ガスト
def wind_mean(z, prof):
    base=np.array(prof["wind_vector"])
    return base+np.array(prof["gradient"])*z if prof["type"]=="altitude" else base

def make_interp(table, default):
    if not table: return lambda t: default
    x,y=zip(*table); return interp1d(x,y,bounds_error=False,fill_value=(y[0],y[-1]))

# ───── Dryden ガスト: 一次オイラー近似 (Shinozuka 法に近似)
class DrydenGust:
    def __init__(self, sigma:float, L:float, dt:float):
        self.sigma=sigma; self.L=L; self.dt=dt
        self.v=0.0  # 差分方程式状態
    def step(self, V_cmd:float):  # V_cmd = 風速 (ローカル流れ) [m/s] (不要でも interface 合わせ)
        # 1次フィルタ:  τ = L / V_cmd≈infty→L/10 として安定化
        V= max(abs(V_cmd), 0.1)
        tau = self.L / V
        a = self.dt / tau
        # ホワイトノイズ入力
        wn = random.gauss(0, self.sigma*math.sqrt(2/tau))
        self.v += -a*self.v + wn*math.sqrt(self.dt)
        return self.v

# ────────────────────────────────────── メイン

def main():
    cfg=json.load(open("config.json",encoding="utf-8"))

    # STL 幾何
    mesh=trimesh.load(cfg["stl_path"]); mesh.apply_scale(cfg.get("stl_scale",1))
    area = mesh.area_faces[np.abs(mesh.face_normals[:,2])>0.9].sum()
    radius=math.sqrt(area/math.pi); L=cfg["rocket_length"]

    # 補間関数
    thrust_fn = make_interp(cfg.get("thrust_curve"), cfg["thrust"])
    mdot_fn   = make_interp(cfg.get("burn_rate_curve"), cfg["mass_propellant"]/cfg["burn_time"])

    # 簡易 Cl/Cy/Cm 線形係数 (small angle)
    def Cl(alpha): return 2*alpha  # rad
    def Cy(beta):  return 2*beta
    Cm_coeff = -0.1
    Cn_coeff = -0.1

    # Dryden ガストインスタンス
    turb=cfg.get("turbulence",{"sigma":[0,0,0],"length":[1,1,1]})
    gust_u=DrydenGust(turb["sigma"][0], turb["length"][0], cfg["dt"])
    gust_v=DrydenGust(turb["sigma"][1], turb["length"][1], cfg["dt"])
    gust_w=DrydenGust(turb["sigma"][2], turb["length"][2], cfg["dt"])

    # 慣性テンソル
    def inertia(m):
        Ixx=(1/12)*m*(3*radius**2+L**2); Izz=0.5*m*radius**2
        return np.array([Ixx,Ixx,Izz])

    # 状態ベクトル [p(3),v(3),q(4),ω(3)]
    state=np.zeros(13); state[6:10]=[0,0,0,1]
    mass=cfg["mass_total"]; fuel=cfg["mass_propellant"]; I=inertia(mass)
    g=9.80665; t=0; h=cfg["dt"]; burn=cfg["burn_time"]

    log=[]

    while state[2]>=0:
        p=state[:3]; v=state[3:6]; q=R.from_quat(state[6:10]); omega=state[10:]
        rho = rho_atm(p[2], cfg["air_density_profile"])

        # 風 + 乱流
        V_mean = wind_mean(p[2], cfg["wind_profile"])
        gust=np.array([gust_u.step(v[0]), gust_v.step(v[0]), gust_w.step(v[0])])
        wind=V_mean+gust
        v_rel = v - wind; V_mag=np.linalg.norm(v_rel)+1e-6

        # 迎角 α, 横滑り角 β
        v_body = q.inv().apply(v_rel)
        alpha = math.atan2(v_body[2], v_body[0])
        beta  = math.asin  (v_body[1]/V_mag)

        # 空力係数
        Cl_val=Cl(alpha); Cy_val=Cy(beta)
        # 力 (機体系)
        L_body = 0.5*rho*V_mag**2*area*Cl_val*np.array([0,1,0])
        Y_body = 0.5*rho*V_mag**2*area*Cy_val*np.array([0,0,1])
        # 慣性系へ
        F_L = q.apply(L_body); F_Y=q.apply(Y_body)

        # 推力
        F_T = thrust_fn(t)*q.apply([0,0,1]) if t<=burn and fuel>0 else np.zeros(3)
        F_D = -0.5*rho*cfg["drag_coefficient"]*area*V_mag**2*normalize(v_rel)
        F_G = np.array([0,0,-mass*g])
        F_total=F_L+F_Y+F_T+F_D+F_G
        a = F_total/mass

        # モーメント (機体系)→慣性系
        M_pitch = 0.5*rho*V_mag**2*area*L*Cm_coeff*alpha  # about Y
        N_yaw   = 0.5*rho*V_mag**2*area*L*Cn_coeff*beta   # about Z
        tau_body=np.array([0, M_pitch, N_yaw])
        tau = q.apply(tau_body) + np.cross(q.apply([0.01,0,0]),F_T)  # + 推力オフセット
        alpha_ang = tau/I

        # Euler integration (簡易)
        state[:3]+=v*h
        state[3:6]+=a*h
        omega+=alpha_ang*h
        state[10:]=omega
        dq = R.from_rotvec(omega*h)
        q = dq*q; q_arr=q.as_quat(); q_arr/=np.linalg.norm(q_arr); state[6:10]=q_arr

        # 質量減少
        dm=mdot_fn(t)*h if t<=burn and fuel>0 else 0
        fuel-=dm; mass-=dm; I[:]=inertia(mass)

        # ログ
        log.append([round(t,3),*np.round(p,3),*np.round(v,3),*np.round(a,3),*np.round(q_arr,4)])
        t+=h

    # CSV 出力
    with open("trajectory_attitude_env.csv","w",newline="",encoding="utf-8") as f:
        csv.writer(f).writerows([["Time(s)","PosX","PosY","PosZ","VelX","VelY","VelZ","AccX","AccY","AccZ","QuatX","QuatY","QuatZ","QuatW"],*log])
    print("✅ 6DoF + Dryden ガスト CSV 出力完了")

if __name__=="__main__":
    main()
