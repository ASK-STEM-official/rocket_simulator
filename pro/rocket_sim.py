import json
import csv
import math
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R

# ----------------------------------------
# normalize: ベクトルを正規化する関数
# ----------------------------------------
def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm != 0 else np.zeros_like(v)

# ----------------------------------------
# get_air_density: 高度 z における空気密度を計算
# ----------------------------------------
def get_air_density(z, profile):
    if profile["type"] == "exponential":
        rho0 = profile["rho0"]
        H    = profile["scale_height"]
        return rho0 * math.exp(-z / H)
    else:
        return profile.get("rho0", 1.225)

# ----------------------------------------
# get_wind: 高度 z における風速ベクトルを計算
# ----------------------------------------
def get_wind(z, profile):
    base = np.array(profile["wind_vector"])
    if profile["type"] == "altitude":
        grad = np.array(profile["gradient"])
        return base + grad * z
    else:
        return base

# ----------------------------------------
# メインシミュレーション
# ----------------------------------------
def main():
    # 設定ファイル読み込み
    with open("config.json", "r") as f:
        config = json.load(f)

    # STL 読み込み＋スケール適用 (mm→m など)
    mesh = trimesh.load(config["stl_path"])
    mesh.apply_scale(config.get("stl_scale", 1.0))

    # プロジェクション断面積（Z軸方向）
    if hasattr(mesh, "area_faces"):
        normals = mesh.face_normals
        areas   = mesh.area_faces
        area = areas[np.abs(normals[:,2]) > 0.9].sum()
    else:
        b = mesh.bounds
        area = (b[1][0]-b[0][0]) * (b[1][1]-b[0][1])
    print(f"使用断面積: {area:.6f} m²")

    # モデル寸法取得（ロケット長さは config["rocket_length"]）
    b = mesh.bounds
    raw_height = b[1][2] - b[0][2]  # モデル単位の高さ
    rocket_length = config["rocket_length"]
    # 高さスケールの検算
    if raw_height > 0:
        print(f"STLモデル高さ: {raw_height:.6f} m → 実寸全長: {rocket_length:.6f} m")

    # 半径を断面積から逆算 (円筒近似)
    radius = math.sqrt(area / math.pi)

    # 物理定数・パラメータ
    g        = 9.80665
    mass     = config["mass_total"]
    fuel     = config["mass_propellant"]
    burn_time= config["burn_time"]
    thrust_N = config["thrust"]
    Cd       = config["drag_coefficient"]
    dt       = config["dt"]

    # 慣性モーメント（円筒体：軸は Z 方向）
    Ixx = (1/12) * mass * (3*radius**2 + rocket_length**2)
    Iyy = Ixx
    Izz =  0.5 * mass * radius**2
    I    = np.array([Ixx, Iyy, Izz])

    # 発射角度 → 単位方向ベクトル
    theta = math.radians(config["launch_angle_deg"])
    phi   = math.radians(config["launch_azimuth_deg"])
    init_dir = normalize([
        math.cos(theta)*math.sin(phi),
        math.cos(theta)*math.cos(phi),
        math.sin(theta)
    ])

    # 初期状態
    time = 0.0
    pos  = np.zeros(3)  # x,y,z
    vel  = np.zeros(3)
    q    = R.from_quat([0,0,0,1])  # 姿勢 (四元数)
    omega= np.zeros(3)             # 角速度

    # ログ用
    log = []

    # シミュレーションループ：地面（z>=0）まで
    while pos[2] >= 0:
        rho  = get_air_density(pos[2], config["air_density_profile"])
        wind = get_wind(pos[2],    config["wind_profile"])

        # 姿勢→推力方向
        thrust_dir = q.apply([0,0,1])

        # 推力と質量減少
        if time <= burn_time and fuel > 0:
            thrust = thrust_N * thrust_dir
            burn_rate = config["mass_propellant"] / burn_time
            dm = burn_rate * dt
            fuel -= dm
            mass -= dm
        else:
            thrust = np.zeros(3)

        # 力：重力 + 推力 + 空気抵抗
        gravity = np.array([0,0,-mass*g])
        rel_vel = vel - wind
        v_rel   = np.linalg.norm(rel_vel)
        drag    = -0.5 * rho * Cd * area * v_rel**2 * normalize(rel_vel) if v_rel>0 else np.zeros(3)

        # 回転トルク（ノズルオフセット仮定）
        offset = q.apply([0.01,0,0])
        torque = np.cross(offset, thrust)
        alpha  = torque / I
        omega += alpha * dt
        dq     = R.from_rotvec(omega * dt)
        q      = dq * q

        # 運動方程式
        total_force = thrust + gravity + drag
        acc = total_force / mass
        vel += acc * dt
        pos += vel * dt

        # 記録
        log.append([
            round(time,3), *map(lambda x:round(x,3),pos),
            *map(lambda x:round(x,3),vel),
            *map(lambda x:round(x,3),acc),
            round(mass,3), round(fuel,3),
            *np.round(q.as_quat(),4),
            *np.round(omega,4),
            round(np.linalg.norm(thrust),3), round(np.linalg.norm(drag),3),
            round(rho,4), *np.round(wind,4)
        ])

        time += dt

    # CSV に保存
    with open("trajectory_attitude_env.csv","w",newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Time(s)","PosX","PosY","PosZ",
            "VelX","VelY","VelZ",
            "AccX","AccY","AccZ",
            "Mass","Fuel",
            "QuatX","QuatY","QuatZ","QuatW",
            "OmegaX","OmegaY","OmegaZ",
            "Thrust(N)","Drag(N)","AirDensity","WindX","WindY","WindZ"
        ])
        writer.writerows(log)

    print("✅ trajectory_attitude_env.csv に出力しました")

if __name__ == "__main__":
    main()
