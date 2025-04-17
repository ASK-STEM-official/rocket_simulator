import tkinter as tk
from tkinter import filedialog, messagebox
import json
import os
import trimesh

class ConfigEditorJP:
    def __init__(self, root):
        self.root = root
        self.root.title("RocketSim 設定エディタ")
        self.entries = {}
        self.filename = ""

        # フィールド定義：キーとラベルのペア
        self.fields = [
            ("stl_path", "STLファイルパス"),
            ("stl_scale", "STLスケール（例：0.001）"),
            ("mass_total", "ロケット全体の質量 (kg)"),
            ("mass_propellant", "燃料質量 (kg)"),
            ("thrust", "推力 (N)"),
            ("burn_time", "燃焼時間 (秒)"),
            ("drag_coefficient", "空気抵抗係数 Cd"),
            ("dt", "時間ステップ dt (秒)"),
            ("launch_angle_deg", "発射角 (°)"),
            ("launch_azimuth_deg", "方位角 (°)"),
            ("rocket_length", "ロケット全長 (m)")  # 全長入力項目
        ]

        self.air_types = ["constant", "exponential"]
        self.wind_types = ["constant", "altitude"]

        self._build_gui()

    def _build_gui(self):
        frame = tk.Frame(self.root)
        frame.pack(padx=10, pady=10)

        # 入力フィールド生成
        for i, (key, label) in enumerate(self.fields):
            tk.Label(frame, text=label).grid(row=i, column=0, sticky="e")
            entry = tk.Entry(frame, width=35)
            entry.grid(row=i, column=1)
            self.entries[key] = entry

        # STLスケール確認ボタン
        tk.Button(frame, text="STLスケール確認", command=self.show_stl_info).grid(row=0, column=2, padx=5)
        # ロケット全長からスケール計算ボタン
        tk.Button(frame, text="全長からスケール計算", command=self.calculate_scale).grid(row=1, column=2, padx=5)

        # 空気密度プロファイル
        base = len(self.fields)
        tk.Label(frame, text="空気密度プロファイルの種類").grid(row=base, column=0, sticky="e")
        self.air_var = tk.StringVar()
        tk.OptionMenu(frame, self.air_var, *self.air_types).grid(row=base, column=1, sticky="w")

        # rho0, スケールハイト
        self.rho0_entry = self._make_labeled_entry(frame, "初期密度 ρ0 (kg/m³)", base+1)
        self.scale_height_entry = self._make_labeled_entry(frame, "スケールハイト H (m)", base+2)

        # 風プロファイル
        tk.Label(frame, text="風プロファイルの種類").grid(row=base+3, column=0, sticky="e")
        self.wind_var = tk.StringVar()
        tk.OptionMenu(frame, self.wind_var, *self.wind_types).grid(row=base+3, column=1, sticky="w")

        # 初期風速ベクトル, 勾配
        self.wind_vec_entry = self._make_labeled_entry(frame, "初期風速ベクトル (x,y,z)", base+4)
        self.gradient_entry = self._make_labeled_entry(frame, "風速勾配ベクトル (x,y,z)", base+5)

        # ボタン
        tk.Button(frame, text="設定ファイルを開く", command=self.load_config).grid(row=base+6, column=0, pady=10)
        tk.Button(frame, text="保存する", command=self.save_config).grid(row=base+6, column=1, pady=10)

    def _make_labeled_entry(self, frame, label, row):
        tk.Label(frame, text=label).grid(row=row, column=0, sticky="e")
        entry = tk.Entry(frame, width=35)
        entry.grid(row=row, column=1)
        return entry

    def load_config(self):
        self.filename = filedialog.askopenfilename(filetypes=[("JSONファイル", "*.json")])
        if not self.filename:
            return
        with open(self.filename, "r", encoding="utf-8") as f:
            config = json.load(f)
        for key, _ in self.fields:
            value = config.get(key, "")
            self.entries[key].delete(0, tk.END)
            self.entries[key].insert(0, str(value))
        # 空気密度
        air = config.get("air_density_profile", {})
        self.air_var.set(air.get("type", "constant"))
        self.rho0_entry.delete(0, tk.END)
        self.rho0_entry.insert(0, str(air.get("rho0", "")))
        self.scale_height_entry.delete(0, tk.END)
        self.scale_height_entry.insert(0, str(air.get("scale_height", "")))
        # 風プロファイル
        wind = config.get("wind_profile", {})
        self.wind_var.set(wind.get("type", "constant"))
        self.wind_vec_entry.delete(0, tk.END)
        self.wind_vec_entry.insert(0, ",".join(map(str, wind.get("wind_vector", [0,0,0]))))
        self.gradient_entry.delete(0, tk.END)
        self.gradient_entry.insert(0, ",".join(map(str, wind.get("gradient", [0,0,0]))))

    def save_config(self):
        if not self.filename:
            self.filename = filedialog.asksaveasfilename(defaultextension=".json")
            if not self.filename:
                return
        config = {}
        for key, _ in self.fields:
            val = self.entries[key].get()
            try:
                config[key] = float(val) if "." in val or "e" in val else int(val)
            except:
                config[key] = val
        config["air_density_profile"] = {
            "type": self.air_var.get(),
            "rho0": float(self.rho0_entry.get()),
            "scale_height": float(self.scale_height_entry.get())
        }
        wind_vec = list(map(float, self.wind_vec_entry.get().split(",")))
        grad = list(map(float, self.gradient_entry.get().split(",")))
        config["wind_profile"] = {
            "type": self.wind_var.get(),
            "wind_vector": wind_vec,
            "gradient": grad
        }
        with open(self.filename, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        messagebox.showinfo("保存完了", f"{os.path.basename(self.filename)} を保存しました。")

    def show_stl_info(self):
        stl_path = self.entries["stl_path"].get()
        try:
            stl_scale = float(self.entries["stl_scale"].get())
        except:
            messagebox.showerror("エラー", "STLスケールの値が不正です。数値で入力してください。")
            return
        if not os.path.isfile(stl_path):
            messagebox.showerror("エラー", f"指定されたSTLファイルが存在しません:\n{stl_path}")
            return
        try:
            mesh = trimesh.load(stl_path)
            mesh.apply_scale(stl_scale)
            volume = mesh.volume
            area = mesh.area
            bounds = mesh.bounds
            cross_section = (bounds[1][0] - bounds[0][0]) * (bounds[1][1] - bounds[0][1])
            info = (
                f"体積: {volume:.6f} m³\n"
                f"表面積: {area:.6f} m²\n"
                f"断面積(XY): {cross_section:.6f} m²\n"
                f"寸法 (幅×奥行×高さ): "
                f"{bounds[1][0] - bounds[0][0]:.3f} × "
                f"{bounds[1][1] - bounds[0][1]:.3f} × "
                f"{bounds[1][2] - bounds[0][2]:.3f} m"
            )
            messagebox.showinfo("STLスケール情報", info)
        except Exception as e:
            messagebox.showerror("エラー", f"STL解析エラー:\n{e}")

    def calculate_scale(self):
        # ロケット全長からSTLスケールを計算
        stl_path = self.entries["stl_path"].get()
        length_str = self.entries["rocket_length"].get()
        try:
            rocket_length = float(length_str)
        except:
            messagebox.showerror("エラー", "ロケット全長の値が不正です。数値で入力してください。")
            return
        if not os.path.isfile(stl_path):
            messagebox.showerror("エラー", f"指定されたSTLファイルが存在しません:\n{stl_path}")
            return
        try:
            mesh = trimesh.load(stl_path)
            raw_bounds = mesh.bounds
            raw_height = raw_bounds[1][2] - raw_bounds[0][2]
            if raw_height == 0:
                raise ValueError("STLモデルの高さが 0 です。")
            scale = rocket_length / raw_height
            # 計算結果を stl_scale エントリに反映
            self.entries["stl_scale"].delete(0, tk.END)
            self.entries["stl_scale"].insert(0, f"{scale:.6f}")
            messagebox.showinfo("計算完了", f"計算されたSTLスケール: {scale:.6f}")
        except Exception as e:
            messagebox.showerror("エラー", f"STL解析エラー:\n{e}")

# === 実行 ===
if __name__ == "__main__":
    root = tk.Tk()
    app = ConfigEditorJP(root)
    root.mainloop()
