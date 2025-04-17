import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import json
import os
import trimesh

class ConfigEditorJP:
    """RocketSim 6DoF + Turbulence 用 JSON 設定エディタ"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("RocketSim 設定エディタ 6DoF + Turbulence")
        self.entries: dict[str, tk.Entry] = {}
        self.filename = ""

        # ───────── 基本パラメータ (key, label) ─────────
        self.base_fields = [
            ("stl_path",           "STLファイルパス"),
            ("stl_scale",          "STLスケール (例 0.001)"),
            ("mass_total",         "ロケット全質量 (kg)"),
            ("mass_propellant",    "燃料質量 (kg)"),
            ("thrust",             "推力定数 (N)"),
            ("burn_time",          "燃焼時間 (s)"),
            ("drag_coefficient",   "Cd (定数)"),
            ("dt",                 "時間ステップ dt (s)"),
            ("launch_angle_deg",   "発射角 (°)"),
            ("launch_azimuth_deg", "方位角 (°)"),
            ("rocket_length",      "ロケット全長 (m)")
        ]
        self.air_types  = ["constant", "exponential"]
        self.wind_types = ["constant", "altitude"]
        self._build_gui()

    # ───────────────── GUI 構築 ─────────────────
    def _build_gui(self):
        frm = ttk.Frame(self.root)
        frm.pack(padx=10, pady=10)

        # 基本フィールド
        for i, (key, label) in enumerate(self.base_fields):
            ttk.Label(frm, text=label).grid(row=i, column=0, sticky="e")
            ent = ttk.Entry(frm, width=38)
            ent.grid(row=i, column=1)
            self.entries[key] = ent
        ttk.Button(frm, text="STLスケール確認", command=self.show_stl_info).grid(row=0, column=2, padx=4)
        ttk.Button(frm, text="全長→スケール",   command=self.calculate_scale).grid(row=1, column=2, padx=4)

        base = len(self.base_fields)
        # 空気密度
        ttk.Label(frm, text="空気密度プロファイル").grid(row=base, column=0, sticky="e")
        self.air_var = tk.StringVar(value="exponential")
        ttk.OptionMenu(frm, self.air_var, "exponential", *self.air_types).grid(row=base, column=1, sticky="w")
        self.rho0_entry = self._add_entry(frm, "ρ0 (kg/m³)", base+1)
        self.H_entry   = self._add_entry(frm, "スケールハイト H (m)", base+2)

        # 風
        ttk.Label(frm, text="風プロファイル").grid(row=base+3, column=0, sticky="e")
        self.wind_var = tk.StringVar(value="altitude")
        ttk.OptionMenu(frm, self.wind_var, "altitude", *self.wind_types).grid(row=base+3, column=1, sticky="w")
        self.wind_entry = self._add_entry(frm, "基準風速 (x,y,z)", base+4)
        self.grad_entry = self._add_entry(frm, "風速勾配 (x,y,z)", base+5)

        # 乱流
        self.sigma_entry = self._add_entry(frm, "乱流 σ (u,v,w)", base+6)
        self.L_entry    = self._add_entry(frm, "乱流長 L (u,v,w)", base+7)

        # 曲線 & Cd
        self.thrust_curve = self._add_entry(frm, "推力曲線 t,N |区切", base+8)
        self.mdot_curve   = self._add_entry(frm, "燃料曲線 t,kg/s|",  base+9)
        self.cd_table     = self._add_entry(frm, "Cd(Mach) M,Cd |",   base+10)

        # ファイル操作
        ttk.Button(frm, text="設定ファイルを開く", command=self.load_config).grid(row=base+11, column=0, pady=10)
        ttk.Button(frm, text="保存", command=self.save_config).grid(row=base+11, column=1, pady=10)

    def _add_entry(self, frame, label, row):
        ttk.Label(frame, text=label).grid(row=row, column=0, sticky="e")
        ent = ttk.Entry(frame, width=38)
        ent.grid(row=row, column=1)
        return ent

    # ─────────────── JSON I/O ───────────────
    def _pairs_to_str(self, pairs):
        return "|".join(["%g,%g" % tuple(p) for p in pairs])

    def _str_to_pairs(self, text):
        result = []
        for seg in text.split("|"):
            if "," in seg:
                try:
                    x, y = map(float, seg.split(","))
                    result.append([x, y])
                except ValueError:
                    pass
        return result

    def load_config(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        self.filename = path
        cfg = json.load(open(path, encoding="utf-8"))

        for k, _ in self.base_fields:
            self.entries[k].delete(0, tk.END)
            self.entries[k].insert(0, str(cfg.get(k, "")))

        air = cfg.get("air_density_profile", {})
        self.air_var.set(air.get("type", "exponential"))
        self.rho0_entry.delete(0, tk.END); self.rho0_entry.insert(0, str(air.get("rho0", "")))
        self.H_entry.delete(0, tk.END); self.H_entry.insert(0, str(air.get("scale_height", "")))

        wind = cfg.get("wind_profile", {})
        self.wind_var.set(wind.get("type", "altitude"))
        self.wind_entry.delete(0, tk.END); self.wind_entry.insert(0, ",".join(map(str, wind.get("wind_vector", [0,0,0]))))
        self.grad_entry.delete(0, tk.END); self.grad_entry.insert(0, ",".join(map(str, wind.get("gradient", [0,0,0]))))

        turb = cfg.get("turbulence", {"sigma": [0,0,0], "length": [1,1,1]})
        self.sigma_entry.delete(0, tk.END); self.sigma_entry.insert(0, ",".join(map(str, turb.get("sigma", [0,0,0]))))
        self.L_entry.delete(0, tk.END); self.L_entry.insert(0, ",".join(map(str, turb.get("length", [1,1,1]))))

        self.thrust_curve.delete(0, tk.END); self.thrust_curve.insert(0, self._pairs_to_str(cfg.get("thrust_curve", [])))
        self.mdot_curve.delete(0, tk.END);   self.mdot_curve.insert(0, self._pairs_to_str(cfg.get("burn_rate_curve", [])))
        self.cd_table.delete(0, tk.END);     self.cd_table.insert(0, self._pairs_to_str(cfg.get("Cd_table", [])))

    def save_config(self):
        if not self.filename:
            self.filename = filedialog.asksaveasfilename(defaultextension=".json")
            if not self.filename:
                return
        cfg = {}
        for k, _ in self.base_fields:
            txt = self.entries[k].get()
            try:
                cfg[k] = float(txt) if ("." in txt or "e" in txt.lower()) else int(txt)
            except ValueError:
                cfg[k] = txt

        cfg["air_density_profile"] = {
            "type": self.air_var.get(),
            "rho0": float(self.rho0_entry.get()),
            "scale_height": float(self.H_entry.get())
        }
        cfg["wind_profile"] = {
            "type": self.wind_var.get(),
            "wind_vector": list(map(float, self.wind_entry.get().split(",")) if self.wind_entry.get() else [0,0,0]),
            "gradient":    list(map(float, self.grad_entry.get().split(",")) if self.grad_entry.get() else [0,0,0])
        }
        cfg["turbulence"] = {
            "sigma":  list(map(float, self.sigma_entry.get().split(",")) if self.sigma_entry.get() else [0,0,0]),
            "length": list(map(float, self.L_entry.get().split(",")) if self.L_entry.get() else [1,1,1])
        }
        cfg["thrust_curve"]    = self._str_to_pairs(self.thrust_curve.get())
        cfg["burn_rate_curve"] = self._str_to_pairs(self.mdot_curve.get())
        cfg["Cd_table"]        = self._str_to_pairs(self.cd_table.get())

        with open(self.filename, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        messagebox.showinfo("保存", f"{os.path.basename(self.filename)} を保存しました。")

    # ─────────────── STL 情報表示 ───────────────
    def show_stl_info(self):
        stl_path = self.entries["stl_path"].get()
        try:
            mesh = trimesh.load(stl_path)
            scale = float(self.entries["stl_scale"].get())
            mesh.apply_scale(scale)
            volume = mesh.volume
            area = mesh.area
            bounds = mesh.bounds
            cross_section = (bounds[1][0]-bounds[0][0])*(bounds[1][1]-bounds[0][1])
            dims = bounds[1] - bounds[0]
            info = (
                f"体積: {volume:.6f} m³\n"
                f"表面積: {area:.6f} m²\n"
                f"断面積(XY): {cross_section:.6f} m²\n"
                f"寸法 (幅×奥行×高さ): {dims[0]:.3f} × {dims[1]:.3f} × {dims[2]:.3f} m"
            )
            messagebox.showinfo("STLスケール情報", info)
        except Exception as e:
            messagebox.showerror("エラー", f"STL解析エラー:\n{e}")

    # ─────────────── 全長からスケール計算 ───────────────
    def calculate_scale(self):
        stl_path = self.entries["stl_path"].get()
        try:
            rocket_length = float(self.entries["rocket_length"].get())
        except ValueError:
            messagebox.showerror("エラー", "ロケット全長の値が不正です。")
            return
        if not os.path.isfile(stl_path):
            messagebox.showerror("エラー", f"指定されたSTLファイルが存在しません:\n{stl_path}")
            return
        try:
            mesh = trimesh.load(stl_path)
            raw_height = mesh.bounds[1][2] - mesh.bounds[0][2]
            if raw_height == 0:
                raise ValueError("STLモデルの高さが 0 です。")
            scale = rocket_length / raw_height
            self.entries["stl_scale"].delete(0, tk.END)
            self.entries["stl_scale"].insert(0, f"{scale:.6f}")
            messagebox.showinfo("計算完了", f"計算されたSTLスケール: {scale:.6f}")
        except Exception as e:
            messagebox.showerror("エラー", f"STL解析エラー:\n{e}")

# ─────────────── メインループ ───────────────
if __name__ == "__main__":
    root = tk.Tk()
    app = ConfigEditorJP(root)
    root.mainloop()
