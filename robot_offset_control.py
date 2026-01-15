import tkinter as tk
import math
import time

# =============================================================================
# PARAMÈTRES DU ROBOT
# =============================================================================
WHEEL_DIAMETER = 6.430  # cm
WHEEL_RADIUS = WHEEL_DIAMETER / 2  # cm
WHEELBASE = 13.34  # cm (entraxe)

# Commandes moteur
PWM_MIN = 145
PWM_MAX = 255

# Distance du point offset devant le robot (paramètre clé!)
OFFSET_DISTANCE = 10.0  # cm - à ajuster pour voir l'effet

# Gain du contrôleur proportionnel
K_GAIN = 2.0

# Vitesse max du point P (saturation pour éviter l'explosion)
MAX_P_SPEED = 30.0  # cm/s

# Paramètres de simulation
DT = 0.05  # pas de temps (secondes)
SCALE = 3  # pixels par cm

# =============================================================================
# CLASSE ROBOT
# =============================================================================
class DifferentialRobot:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x  # position x (cm)
        self.y = y  # position y (cm)
        self.theta = theta  # orientation (radians)

        # Vitesses actuelles
        self.v = 0  # vitesse linéaire (cm/s)
        self.omega = 0  # vitesse angulaire (rad/s)

        # Commandes PWM des moteurs
        self.pwm_left = 0
        self.pwm_right = 0

    def get_offset_point(self, b):
        """Calcule la position du point virtuel P devant le robot"""
        Px = self.x + b * math.cos(self.theta)
        Py = self.y + b * math.sin(self.theta)
        return Px, Py

    def compute_control(self, target_x, target_y, b, k, max_speed):
        """
        Contrôleur par point offset

        1. Calcule où est le point P
        2. Calcule comment P doit bouger pour aller vers l'objectif
        3. Inverse la matrice pour trouver v et omega
        """
        # Position du point offset
        Px, Py = self.get_offset_point(b)

        # Erreur vers l'objectif
        error_x = target_x - Px
        error_y = target_y - Py

        # Distance à l'objectif
        distance = math.sqrt(error_x**2 + error_y**2)

        # Vitesse désirée du point P (loi proportionnelle)
        Pdot_x_desired = k * error_x
        Pdot_y_desired = k * error_y

        # SATURATION : limiter la vitesse du point P
        speed_desired = math.sqrt(Pdot_x_desired**2 + Pdot_y_desired**2)
        if speed_desired > max_speed:
            # Normaliser pour garder la direction mais limiter la norme
            scale = max_speed / speed_desired
            Pdot_x_desired *= scale
            Pdot_y_desired *= scale

        # Inversion de la matrice pour trouver v et omega
        # [v]     [cos(θ)    sin(θ)  ] [Pdot_x]
        # [ω]  =  [-sin(θ)/b cos(θ)/b] [Pdot_y]

        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        v = cos_theta * Pdot_x_desired + sin_theta * Pdot_y_desired
        omega = (-sin_theta / b) * Pdot_x_desired + (cos_theta / b) * Pdot_y_desired

        return v, omega, distance

    def velocity_to_pwm(self, v, omega):
        """
        Convertit (v, omega) en commandes PWM pour les moteurs

        Cinématique différentielle:
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / L

        Donc:
        v_left = v - (L/2) * omega
        v_right = v + (L/2) * omega
        """
        L = WHEELBASE

        # Vitesses des roues (cm/s)
        v_left = v - (L / 2) * omega
        v_right = v + (L / 2) * omega

        # Vitesse max théorique (on suppose PWM_MAX = vitesse max)
        # On normalise pour que PWM_MAX corresponde à une vitesse raisonnable
        MAX_WHEEL_SPEED = 50  # cm/s à PWM_MAX (à calibrer selon ton robot)

        # Conversion en PWM
        def speed_to_pwm(speed):
            if abs(speed) < 1:  # Zone morte
                return 0

            # Normalisation
            normalized = speed / MAX_WHEEL_SPEED
            normalized = max(-1, min(1, normalized))  # Clamp

            # Mapping vers PWM
            if normalized > 0:
                pwm = PWM_MIN + (PWM_MAX - PWM_MIN) * normalized
            else:
                pwm = -PWM_MIN + (-PWM_MAX + PWM_MIN) * normalized

            return int(pwm)

        pwm_left = speed_to_pwm(v_left)
        pwm_right = speed_to_pwm(v_right)

        return pwm_left, pwm_right, v_left, v_right

    def update(self, v, omega, dt):
        """Met à jour la position du robot (simulation)"""
        self.v = v
        self.omega = omega

        # Intégration (modèle cinématique)
        if abs(omega) < 0.001:
            # Mouvement rectiligne
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            # Mouvement circulaire
            self.x += (v / omega) * (math.sin(self.theta + omega * dt) - math.sin(self.theta))
            self.y += (v / omega) * (-math.cos(self.theta + omega * dt) + math.cos(self.theta))
            self.theta += omega * dt

        # Normalise theta entre -pi et pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))


# =============================================================================
# INTERFACE GRAPHIQUE
# =============================================================================
class RobotSimulator:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Simulateur Robot - Contrôle par Point Offset")

        # Canvas
        self.canvas_width = 800
        self.canvas_height = 600
        self.canvas = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg='white')
        self.canvas.pack(side=tk.LEFT)

        # Panneau de contrôle
        self.control_frame = tk.Frame(self.root, padx=10, pady=10)
        self.control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # Robot (centré dans le canvas)
        center_x = self.canvas_width / (2 * SCALE)
        center_y = self.canvas_height / (2 * SCALE)
        self.robot = DifferentialRobot(x=center_x, y=center_y, theta=0)

        # Objectif
        self.target_x = None
        self.target_y = None
        self.running = False

        # Trajectoire
        self.trajectory = []

        # Création des contrôles
        self.create_controls()

        # Bindingcanvas
        self.canvas.bind("<Button-1>", self.on_click)

        # Démarrer la boucle de simulation
        self.simulate()

    def create_controls(self):
        """Crée le panneau de contrôle"""
        tk.Label(self.control_frame, text="PARAMÈTRES", font=('Arial', 12, 'bold')).pack(pady=10)

        # Offset distance
        tk.Label(self.control_frame, text="Distance offset (b):").pack()
        self.offset_var = tk.DoubleVar(value=OFFSET_DISTANCE)
        self.offset_scale = tk.Scale(self.control_frame, from_=1, to=30,
                                      variable=self.offset_var, orient=tk.HORIZONTAL, length=150)
        self.offset_scale.pack()

        # Gain K
        tk.Label(self.control_frame, text="Gain K:").pack()
        self.k_var = tk.DoubleVar(value=K_GAIN)
        self.k_scale = tk.Scale(self.control_frame, from_=0.5, to=5, resolution=0.1,
                                 variable=self.k_var, orient=tk.HORIZONTAL, length=150)
        self.k_scale.pack()

        # Vitesse max du point P
        tk.Label(self.control_frame, text="Vitesse max P (cm/s):").pack()
        self.max_speed_var = tk.DoubleVar(value=MAX_P_SPEED)
        self.max_speed_scale = tk.Scale(self.control_frame, from_=10, to=100, resolution=5,
                                         variable=self.max_speed_var, orient=tk.HORIZONTAL, length=150)
        self.max_speed_scale.pack()

        tk.Label(self.control_frame, text="").pack(pady=10)
        tk.Label(self.control_frame, text="ÉTAT ROBOT", font=('Arial', 12, 'bold')).pack()

        # Affichage état
        self.state_label = tk.Label(self.control_frame, text="", justify=tk.LEFT, font=('Courier', 9))
        self.state_label.pack(pady=5)

        # Affichage PWM
        tk.Label(self.control_frame, text="").pack(pady=5)
        tk.Label(self.control_frame, text="COMMANDES PWM", font=('Arial', 12, 'bold')).pack()
        self.pwm_label = tk.Label(self.control_frame, text="", justify=tk.LEFT, font=('Courier', 10))
        self.pwm_label.pack(pady=5)

        # Boutons
        tk.Label(self.control_frame, text="").pack(pady=10)
        tk.Button(self.control_frame, text="Reset Position", command=self.reset).pack(pady=5)
        tk.Button(self.control_frame, text="Effacer Trajectoire", command=self.clear_trajectory).pack(pady=5)

        # Instructions
        tk.Label(self.control_frame, text="").pack(pady=20)
        tk.Label(self.control_frame, text="INSTRUCTIONS", font=('Arial', 10, 'bold')).pack()
        tk.Label(self.control_frame, text="Cliquez sur le canvas\npour définir un objectif",
                 justify=tk.CENTER).pack()

        # Légende
        tk.Label(self.control_frame, text="").pack(pady=20)
        tk.Label(self.control_frame, text="LÉGENDE", font=('Arial', 10, 'bold')).pack()
        tk.Label(self.control_frame, text="🔵 Robot", fg='blue').pack(anchor='w')
        tk.Label(self.control_frame, text="🟢 Point offset P", fg='green').pack(anchor='w')
        tk.Label(self.control_frame, text="🔴 Objectif", fg='red').pack(anchor='w')
        tk.Label(self.control_frame, text="⚫ Trajectoire", fg='gray').pack(anchor='w')

    def on_click(self, event):
        """Gère le clic sur le canvas"""
        # Convertir pixels en cm
        self.target_x = event.x / SCALE
        self.target_y = event.y / SCALE
        self.running = True

    def reset(self):
        """Remet le robot au centre"""
        self.robot.x = self.canvas_width / (2 * SCALE)
        self.robot.y = self.canvas_height / (2 * SCALE)
        self.robot.theta = 0
        self.robot.v = 0
        self.robot.omega = 0
        self.target_x = None
        self.target_y = None
        self.running = False
        self.clear_trajectory()

    def clear_trajectory(self):
        """Efface la trajectoire"""
        self.trajectory = []

    def to_canvas(self, x, y):
        """Convertit coordonnées robot en coordonnées canvas"""
        return x * SCALE, y * SCALE

    def draw(self):
        """Dessine le robot et l'environnement"""
        self.canvas.delete("all")

        # Grille
        for i in range(0, self.canvas_width, 50):
            self.canvas.create_line(i, 0, i, self.canvas_height, fill='#eee')
        for i in range(0, self.canvas_height, 50):
            self.canvas.create_line(0, i, self.canvas_width, i, fill='#eee')

        # Trajectoire
        if len(self.trajectory) > 1:
            for i in range(1, len(self.trajectory)):
                x1, y1 = self.to_canvas(*self.trajectory[i-1])
                x2, y2 = self.to_canvas(*self.trajectory[i])
                self.canvas.create_line(x1, y1, x2, y2, fill='gray', width=2)

        # Objectif
        if self.target_x is not None:
            tx, ty = self.to_canvas(self.target_x, self.target_y)
            self.canvas.create_oval(tx-8, ty-8, tx+8, ty+8, fill='red', outline='darkred')
            self.canvas.create_text(tx, ty-20, text="Objectif", fill='red')

        # Robot
        rx, ry = self.to_canvas(self.robot.x, self.robot.y)
        robot_size = WHEELBASE * SCALE / 2

        # Corps du robot (rectangle orienté)
        cos_t = math.cos(self.robot.theta)
        sin_t = math.sin(self.robot.theta)

        # Points du rectangle
        w = robot_size
        h = robot_size * 0.6
        points = [
            (rx + w*cos_t - h*sin_t, ry + w*sin_t + h*cos_t),
            (rx - w*cos_t - h*sin_t, ry - w*sin_t + h*cos_t),
            (rx - w*cos_t + h*sin_t, ry - w*sin_t - h*cos_t),
            (rx + w*cos_t + h*sin_t, ry + w*sin_t - h*cos_t),
        ]
        self.canvas.create_polygon(points, fill='#4a90d9', outline='#2c5aa0', width=2)

        # Flèche de direction
        arrow_len = robot_size * 1.2
        ax = rx + arrow_len * cos_t
        ay = ry + arrow_len * sin_t
        self.canvas.create_line(rx, ry, ax, ay, fill='#2c5aa0', width=3, arrow=tk.LAST)

        # Roues
        wheel_w = 8
        wheel_h = 4
        for side in [-1, 1]:
            wx = rx + side * (h * 0.8) * (-sin_t)
            wy = ry + side * (h * 0.8) * cos_t
            self.canvas.create_oval(wx-wheel_w, wy-wheel_h, wx+wheel_w, wy+wheel_h, fill='black')

        # Point offset P
        b = self.offset_var.get()
        Px, Py = self.robot.get_offset_point(b)
        px, py = self.to_canvas(Px, Py)
        self.canvas.create_oval(px-6, py-6, px+6, py+6, fill='#00cc00', outline='#009900')
        self.canvas.create_text(px, py-15, text="P", fill='green', font=('Arial', 10, 'bold'))

        # Ligne robot -> P
        self.canvas.create_line(rx, ry, px, py, fill='green', dash=(4,2))

        # Ligne P -> objectif (si objectif défini)
        if self.target_x is not None:
            tx, ty = self.to_canvas(self.target_x, self.target_y)
            self.canvas.create_line(px, py, tx, ty, fill='orange', dash=(4,2), width=2)

    def simulate(self):
        """Boucle de simulation"""
        if self.running and self.target_x is not None:
            b = self.offset_var.get()
            k = self.k_var.get()
            max_speed = self.max_speed_var.get()

            # Calcul du contrôle
            v, omega, distance = self.robot.compute_control(
                self.target_x, self.target_y, b, k, max_speed
            )

            # Conversion en PWM
            pwm_left, pwm_right, v_left, v_right = self.robot.velocity_to_pwm(v, omega)
            self.robot.pwm_left = pwm_left
            self.robot.pwm_right = pwm_right

            # Mise à jour de la position
            self.robot.update(v, omega, DT)

            # Enregistrement trajectoire
            self.trajectory.append((self.robot.x, self.robot.y))
            if len(self.trajectory) > 1000:
                self.trajectory = self.trajectory[-500:]

            # Arrêt si proche de l'objectif
            if distance < 2:
                self.running = False

            # Mise à jour affichage PWM
            self.pwm_label.config(text=f"Gauche: {pwm_left:+4d}\nDroite: {pwm_right:+4d}\n\nv_L: {v_left:+6.1f} cm/s\nv_R: {v_right:+6.1f} cm/s")
        else:
            self.pwm_label.config(text="Gauche:    0\nDroite:    0\n\n(en attente)")

        # Mise à jour affichage état
        theta_deg = math.degrees(self.robot.theta)
        self.state_label.config(text=f"x: {self.robot.x:6.1f} cm\ny: {self.robot.y:6.1f} cm\nθ: {theta_deg:+6.1f}°\n\nv: {self.robot.v:+6.1f} cm/s\nω: {math.degrees(self.robot.omega):+6.1f} °/s")

        # Dessin
        self.draw()

        # Prochaine frame
        self.root.after(int(DT * 1000), self.simulate)

    def run(self):
        """Lance l'application"""
        self.root.mainloop()


# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("SIMULATEUR ROBOT - CONTRÔLE PAR POINT OFFSET")
    print("=" * 60)
    print()
    print("Paramètres du robot:")
    print(f"  - Diamètre roues: {WHEEL_DIAMETER} cm")
    print(f"  - Entraxe: {WHEELBASE} cm")
    print(f"  - PWM: {PWM_MIN} à {PWM_MAX}")
    print()
    print("Cliquez sur le canvas pour définir un objectif!")
    print("Ajustez les paramètres b et K pour voir leur effet.")
    print()
    print("=" * 60)

    sim = RobotSimulator()
    sim.run()
