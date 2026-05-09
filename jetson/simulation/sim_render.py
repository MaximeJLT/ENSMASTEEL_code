import matplotlib.pyplot as plt
from sim_core import WorldState

class Renderer:
    def __init__(self, world: WorldState):
        self.world = world

        self.fig, (self.ax, self.ax_info) = plt.subplots(
            1, 2,
            figsize=(15, 7),
            gridspec_kw={"width_ratios": [3, 1]}
        )

        # --- Axes table ---
        self.ax.set_aspect("equal", adjustable="box")
        table = world.zones.get("table", None)
        if table:
            self.ax.set_xlim(table.x_mm, table.x_mm + table.width_mm)
            self.ax.set_ylim(table.y_mm, table.y_mm + table.height_mm)
        else:
            self.ax.set_xlim(-1600, 1600)
            self.ax.set_ylim(-1100, 1100)
        self.ax.set_title("Simulation Eurobot 2026", fontsize=11)
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")

        # Zones (dessiner une seule fois)
        for z in world.zones.values():
            if z.name == "table":
                color, alpha = "gray", 0.05
            elif z.name == "our_nest":
                color, alpha = "green", 0.15
            elif z.name.startswith("pantry_"):
                color, alpha = "royalblue", 0.18
            else:
                color, alpha = "black", 0.05

            rect = plt.Rectangle(
                (z.x_mm, z.y_mm), z.width_mm, z.height_mm,
                linewidth=1.2, edgecolor=color, facecolor=color, alpha=alpha
            )
            self.ax.add_patch(rect)
            if z.name != "table":
                cx = z.x_mm + z.width_mm / 2
                cy = z.y_mm + z.height_mm / 2
                self.ax.text(cx, cy, z.name, fontsize=6, color=color,
                             ha="center", va="center", alpha=0.9)

        # Strategy path (trace une seule fois)
        if world.strategy_path:
            xs = [p[0] for p in world.strategy_path]
            ys = [p[1] for p in world.strategy_path]
            self.ax.plot(xs, ys, "--", color="gray", linewidth=0.8, alpha=0.5)
            # Marqueurs de waypoints
            self.ax.plot(xs, ys, "x", color="gray", markersize=5, alpha=0.4)

        # Elements dynamiques
        self.robot_plot, = self.ax.plot([], [], marker="o", linestyle="",
                                        markersize=14, color="red", zorder=5)
        self.robot_dir, = self.ax.plot([], [], "-", color="darkred",
                                       linewidth=1.5, zorder=5)
        self.crates_sc = self.ax.scatter([], [], s=70, zorder=4)
        self.cursor_plot, = self.ax.plot([], [], marker="D", linestyle="",
                                         markersize=10, color="purple", zorder=5)
        self.start_plot, = self.ax.plot([], [], "ks", linestyle="", markersize=5, zorder=2)

        # Adversaire
        self.opp_plot, = self.ax.plot([], [], marker="s", linestyle="",
                                      markersize=14, color="darkorange", zorder=5,
                                      label="Adversaire")
        self.opp_dir,  = self.ax.plot([], [], "-", color="saddlebrown",
                                      linewidth=1.5, zorder=5)
        # Cercle de danger (dynamique)
        self._danger_circle = plt.Circle((0, 0), 450, color="orange",
                                         fill=False, linestyle="--",
                                         linewidth=1.0, alpha=0.5, zorder=4)
        self.ax.add_patch(self._danger_circle)

        # Waypoint de détour
        self.detour_plot, = self.ax.plot([], [], marker="*", linestyle="",
                                          markersize=14, color="gold", zorder=6)

        # Labels des IDs de caisses (dynamiques)
        self._crate_texts = {}

        # --- Panneau info ---
        self.ax_info.axis("off")
        self.info_text = self.ax_info.text(
            0.05, 0.97, "",
            transform=self.ax_info.transAxes,
            va="top", ha="left",
            fontsize=8.5,
            fontfamily="monospace"
        )

        # Barre de temps
        self.ax_info.add_patch(plt.Rectangle(
            (0.05, 0.03), 0.90, 0.030,
            transform=self.ax_info.transAxes,
            color="lightgray", zorder=1
        ))
        self.time_bar = self.ax_info.add_patch(plt.Rectangle(
            (0.05, 0.03), 0.0, 0.030,
            transform=self.ax_info.transAxes,
            color="steelblue", zorder=2
        ))
        self.ax_info.text(0.05, 0.065, "0 s", transform=self.ax_info.transAxes,
                          fontsize=7.5, va="bottom")
        self.time_label = self.ax_info.text(
            0.5, 0.065, "", transform=self.ax_info.transAxes,
            fontsize=7.5, va="bottom", ha="center"
        )

        plt.tight_layout()

    # ------------------------------------------------------------------
    def update(self, runner=None):
        import math
        w = self.world

        # Robot
        self.robot_plot.set_data([w.robot.x_mm], [w.robot.y_mm])
        arrow_len = 130
        dx = arrow_len * math.cos(w.robot.theta_rad)
        dy = arrow_len * math.sin(w.robot.theta_rad)
        self.robot_dir.set_data(
            [w.robot.x_mm, w.robot.x_mm + dx],
            [w.robot.y_mm, w.robot.y_mm + dy]
        )

        # Adversaire
        if w.opponent is not None:
            opp = w.opponent
            self.opp_plot.set_data([opp.x_mm], [opp.y_mm])
            odx = arrow_len * math.cos(opp.theta_rad)
            ody = arrow_len * math.sin(opp.theta_rad)
            self.opp_dir.set_data(
                [opp.x_mm, opp.x_mm + odx],
                [opp.y_mm, opp.y_mm + ody]
            )
            self._danger_circle.set_center((opp.x_mm, opp.y_mm))
            self._danger_circle.set_visible(True)
        else:
            self.opp_plot.set_data([], [])
            self.opp_dir.set_data([], [])
            self._danger_circle.set_visible(False)

        # Waypoint de détour (si actif)
        if runner is not None:
            av = runner.avoid_status()
            if av["state"] == "DETOURING" and av["detour_target"]:
                dtx, dty = av["detour_target"]
                self.detour_plot.set_data([dtx], [dty])
            else:
                self.detour_plot.set_data([], [])

        # Caisses
        cxs, cys, colors = [], [], []
        for c in w.crates.values():
            cxs.append(c.x_mm)
            cys.append(c.y_mm)
            if c.delivered:
                colors.append("dimgray")
            elif c.carried:
                colors.append("orange")
            else:
                colors.append("royalblue")

            # Label ID
            if c.id not in self._crate_texts:
                self._crate_texts[c.id] = self.ax.text(
                    c.x_mm, c.y_mm + 18, str(c.id),
                    fontsize=6, ha="center", color="black", zorder=6
                )
            else:
                self._crate_texts[c.id].set_position((c.x_mm, c.y_mm + 18))

        if cxs:
            self.crates_sc.set_offsets(list(zip(cxs, cys)))
            self.crates_sc.set_facecolors(colors)
        else:
            self.crates_sc.set_offsets([])

        # Curseur
        if w.cursor is not None:
            self.cursor_plot.set_data([w.cursor.x_mm], [w.cursor.y_mm])
        else:
            self.cursor_plot.set_data([], [])

        # Start markers
        self.start_plot.set_data(
            [sm.x_mm for sm in w.start_markers],
            [sm.y_mm for sm in w.start_markers]
        )

        # Barre de temps
        progress = min(w.t_s / w.match_duration_s, 1.0)
        self.time_bar.set_width(0.90 * progress)
        self.time_label.set_text(f"{w.t_s:.1f} / {w.match_duration_s:.0f} s")
        if w.match_finished:
            self.time_bar.set_color("crimson")

        # Panneau de score
        bd = w.score_breakdown
        robot = w.robot

        action_labels = {
            "idle":            "En route",
            "picking":         "Ramassage (bras)...",
            "dropping":        "Depose...",
            "dragging_cursor": "Tire le curseur...",
        }
        action_str = action_labels.get(robot.action_state, robot.action_state)

        lines = [
            "MATCH TERMINE" if w.match_finished else "EN COURS",
            "",
            f"SCORE TOTAL : {w.score} pts",
            "",
            f"  Nid        : {bd.get('nest', 0):>3} pts",
        ]

        # Garde-mangers : detail par pantry
        total_pantry_pts = 0
        total_pantry_bonus = 0
        pantry_lines = []
        for i in range(1, 9):
            pname = f"pantry_{i}"
            pts = bd.get(f"pts_{pname}", 0)
            bonus = bd.get(f"bonus_{pname}", 0)
            if pts > 0 or bonus > 0:
                n_crates = pts // 3
                pantry_lines.append(f"  {pname} : {n_crates}c = {pts}+{bonus}pts")
            total_pantry_pts += pts
            total_pantry_bonus += bonus

        lines.append(f"  Garde-mngr : {total_pantry_pts:>3} pts")
        lines.append(f"  Bonus zones: {total_pantry_bonus:>3} pts")
        lines.extend(pantry_lines)
        lines.append(f"  Curseur    : {bd.get('cursor', 0):>3} pts")
        lines.append(f"  Retour nid : {bd.get('return', 0):>3} pts")
        lines.append("")
        lines.append("ROBOT")
        lines.append(f"  x={robot.x_mm:>7.0f} mm")
        lines.append(f"  y={robot.y_mm:>7.0f} mm")
        lines.append(f"  Bras : {len(robot.carried_ids)}/{robot.max_carried} caisses")
        lines.append(f"  IDs  : {robot.carried_ids}")
        lines.append(f"  Actn : {action_str}")
        if robot.action_state == "picking" or robot.action_state == "dropping":
            lines.append(f"  Timer: {robot.action_timer_s:.2f}s")
        lines.append("")
        lines.append("LEGENDE CAISSES")
        lines.append("  \u25cf bleu   = au sol")
        lines.append("  \u25cf orange = portee")
        lines.append("  \u25cf gris   = livree")

        # Adversaire
        if w.opponent is not None:
            opp = w.opponent
            lines.append("")
            lines.append("ADVERSAIRE")
            lines.append(f"  x={opp.x_mm:>7.0f} mm")
            lines.append(f"  y={opp.y_mm:>7.0f} mm")
            dist_to_opp = math.hypot(opp.x_mm - robot.x_mm, opp.y_mm - robot.y_mm)
            lines.append(f"  dist={dist_to_opp:>6.0f} mm")

        # Évitement
        if runner is not None:
            av = runner.avoid_status()
            avoid_icons = {"NORMAL": "OK", "STOPPED": "STOP", "DETOURING": "DETOUR"}
            avoid_colors = {"NORMAL": "NORMAL", "STOPPED": "ARRETE", "DETOURING": "CONTOURNEMENT"}
            lines.append("")
            lines.append("EVITEMENT")
            lines.append(f"  [{avoid_icons.get(av['state'], '?')}] {avoid_colors.get(av['state'], av['state'])}")
            if av["state"] == "DETOURING" and av["detour_target"]:
                dtx, dty = av["detour_target"]
                lines.append(f"  wp=({dtx:.0f},{dty:.0f})")

        self.info_text.set_text("\n".join(lines))

        plt.pause(0.001)
