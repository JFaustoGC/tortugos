import time
import numpy as np
import matplotlib.pyplot as plt
from robot import Robot
from unicycle_controller import UnicycleController
from vision_tracker import VisionTracker
from references import vc_trajectory, follower_reference
from controllers import follower_control

# =============================================================================
# CONFIGURATION
# =============================================================================

CONNECTIVITY = {
    'RAM06': [],  # RAM06 listens to RAM05
    # 'RAM05': ['RAM06'],  # RAM05 listens to RAM06
    # 'RAM02': ['RAM06', 'RAM05'] # Example if RAM02 was active
}

# Physics & Vision
WHEEL_RADIUS = 0.06471 / 2
WHEEL_BASE = 0.07782
UPDATE_RATE = 0.05   # 20 Hz
AREA_WIDTH = 0.49 * 7
AREA_HEIGHT = 0.49 * 4
CAMERA_ROI = (0, 0, 1920, 1080)
START_X, START_Y = 0, 0

# Control Gains
GAINS = {'cx': 1.1, 'ct': 1.0, 'cy': 65.0, 'consensus': 0.00}
K_PARAM = 1.0
TRAJECTORY_DURATION = 1000.0

# Fleet Config
ROBOT_FLEET = [
    {'id': 'RAM06', 'mac': '98:D3:32:20:28:46', 'port': '/dev/rfcomm0', 'marker_id': 7, 'offset': np.array([0.2, 0.2]), 'color': 'blue'},
    # {'id': 'RAM05', 'mac': '98:D3:32:10:15:96', 'port': '/dev/rfcomm1', 'marker_id': 0, 'offset': np.array([-0.2, -0.2]), 'color': 'green'},
    # {'id': 'RAM02', 'mac': '98:D3:32:30:24:38', 'port': '/dev/rfcomm2', 'marker_id': 1, 'offset': np.array([0.0, 0.0]), 'color': 'red'}
]

# =============================================================================
# LOGIC CLASS: FormationRobot
# =============================================================================

class FormationRobot:
    """Handles robot connection, state estimation, and control computation."""
    def __init__(self, config):
        self.id = config['id']
        self.marker_id = config['marker_id']
        self.offset = config['offset']
        self.color = config['color']

        # Hardware
        self.robot = Robot(self.id, config['mac'], config['port'])
        self.controller = UnicycleController(abs(WHEEL_BASE), WHEEL_RADIUS)
        self.connected = False

        # Data Storage (for plotting)
        self.data = {
            't': [], 'x': [], 'y': [],
            'ref_x': [], 'ref_y': [],
            'err_dist': [], 'err_theta': [],
            'v': [], 'w': []
        }

    def connect(self):
        print(f"[{self.id}] Connecting...")
        self.connected = self.robot.connect()
        return self.connected

    def disconnect(self):
        if self.connected:
            self.robot.send_message(self.controller.stop_command())
            time.sleep(0.1)
            self.robot.disconnect()
            print(f"[{self.id}] Disconnected.")

    def update(self, current_time, elapsed_time, vc_state, current_state, all_errors):
        if not self.connected: return

        # Unpack state from Main Loop
        x, y, theta, detected = current_state

        # 1. Recalculate Reference (Needed for v_ref variables)
        x_ref, y_ref, theta_ref, theta_dot_ref, v_ref = follower_reference(vc_state, self.offset)
        x_ref += START_X
        y_ref += START_Y

        # 2. Control Logic
        v_cmd, w_cmd = 0.0, 0.0

        if detected:
            state = [x, y, theta]
            ref = (x_ref, y_ref, theta_ref, theta_dot_ref, v_ref)

            # Get neighbors from global CONNECTIVITY dict
            my_neighbor_list = CONNECTIVITY.get(self.id, [])

            # Compute Control
            v_cmd, w_cmd = follower_control(
                state=state, ref=ref,
                neighbors_errors=all_errors,     # Full error dict
                my_neighbors=my_neighbor_list,   # List of names ['RAM05']
                gains=GAINS, k=K_PARAM
            )

            # Calculate Visual Errors (for plotting only)
            err_dist = np.sqrt((x_ref - x)**2 + (y_ref - y)**2)
            err_theta = np.arctan2(np.sin(theta_ref - theta), np.cos(theta_ref - theta))

            # Store Data
            self._record_data(elapsed_time, x, y, x_ref, y_ref, err_dist, err_theta, v_cmd, w_cmd)

            # Hardware Command (Direct, no filtering)
            command = self.controller.compute_command_from_velocities(v_cmd, w_cmd)
        else:
            command = self.controller.stop_command()
            print(f"[{self.id}] Lost visual tracking")

        # 3. Send Command
        if not self.robot.send_message(command):
            print(f"[{self.id}] Comms lost, reconnecting...")
            self.robot.reconnect()

    def _record_data(self, t, x, y, rx, ry, ed, et, v, w):
        d = self.data
        d['t'].append(t)
        d['x'].append(x); d['y'].append(y)
        d['ref_x'].append(rx); d['ref_y'].append(ry)
        d['err_dist'].append(ed); d['err_theta'].append(np.degrees(et))
        d['v'].append(v); d['w'].append(w)

# =============================================================================
# VISUALIZATION CLASS: FleetVisualizer
# =============================================================================

class FleetVisualizer:
    """Handles all plotting logic. Separates UI from control logic."""
    def __init__(self, robots):
        plt.ion()
        self.fig = plt.figure(figsize=(14, 9))
        self.robots = robots
        self.lines = {}

        # Layout: Left = Trajectory (Big), Right = 3 stacked plots (Errors/Cmds)
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.5, 1])

        # 1. Trajectory Plot (Left Column, Spans all rows)
        self.ax_traj = self.fig.add_subplot(gs[:, 0])
        self.ax_traj.set_title('Formation Trajectory')
        self.ax_traj.set_xlim(-AREA_WIDTH/2, AREA_WIDTH/2)
        self.ax_traj.set_ylim(-AREA_HEIGHT/2, AREA_HEIGHT/2)
        self.ax_traj.set_aspect('equal')
        self.ax_traj.grid(True, alpha=0.3)


# [Image of coordinate system 2d]


        # 2. Dashboard Plots (Right Column)
        self.ax_dist = self.fig.add_subplot(gs[0, 1])
        self.ax_dist.set_title('Position Error (m)')
        self.ax_dist.grid(True)

        self.ax_theta = self.fig.add_subplot(gs[1, 1])
        self.ax_theta.set_title('Heading Error (deg)')
        self.ax_theta.grid(True)

        self.ax_cmd = self.fig.add_subplot(gs[2, 1])
        self.ax_cmd.set_title('Control Inputs (v: solid, w: dashed)')
        self.ax_cmd.grid(True)

        # Initialize Lines per robot
        for r in robots:
            self.init_robot_lines(r)

        self.fig.tight_layout()

    def init_robot_lines(self, robot):
        # Trajectory lines
        trail, = self.ax_traj.plot([], [], '-', color=robot.color, linewidth=1.5, alpha=0.8, label=robot.id)
        ref, = self.ax_traj.plot([], [], '--', color=robot.color, linewidth=1, alpha=0.3)
        head, = self.ax_traj.plot([], [], 'o', color=robot.color)

        # Error lines
        err_d, = self.ax_dist.plot([], [], '-', color=robot.color, label=robot.id)
        err_t, = self.ax_theta.plot([], [], '-', color=robot.color)

        # Command lines
        cmd_v, = self.ax_cmd.plot([], [], '-', color=robot.color, linewidth=1.5)
        cmd_w, = self.ax_cmd.plot([], [], '--', color=robot.color, linewidth=1.0, alpha=0.7)

        self.lines[robot.id] = {
            'trail': trail, 'ref': ref, 'head': head,
            'err_d': err_d, 'err_t': err_t,
            'cmd_v': cmd_v, 'cmd_w': cmd_w
        }

        # Only add legend once
        self.ax_traj.legend(loc='upper right')
        self.ax_dist.legend(loc='upper right')

    def update(self):
        for r in self.robots:
            d = r.data
            if not d['t']: continue

            ln = self.lines[r.id]

            # Update Trajectory
            ln['trail'].set_data(d['x'], d['y'])
            ln['ref'].set_data(d['ref_x'], d['ref_y'])
            ln['head'].set_data([d['x'][-1]], [d['y'][-1]])

            # Update Dashboard
            ln['err_d'].set_data(d['t'], d['err_dist'])
            ln['err_t'].set_data(d['t'], d['err_theta'])
            ln['cmd_v'].set_data(d['t'], d['v'])
            ln['cmd_w'].set_data(d['t'], d['w'])

        # Auto-scale axes (periodically or every frame)
        for ax in [self.ax_dist, self.ax_theta, self.ax_cmd]:
            ax.relim()
            ax.autoscale_view()

        plt.pause(0.001)

    def close(self):
        plt.ioff()
        plt.close()

# =============================================================================
# MAIN EXECUTION
# =============================================================================

if __name__ == "__main__":
    # 1. Initialization
    camera_id = 4  # Adjust based on your setup
    vision = VisionTracker(camera_id, AREA_WIDTH, AREA_HEIGHT, CAMERA_ROI)

    agents = []
    print("Initializing Fleet...")
    for config in ROBOT_FLEET:
        bot = FormationRobot(config)
        if bot.connect():
            agents.append(bot)

    if not agents:
        print("No robots connected.")
        exit()

    # 2. Setup Visualizer
    viz = FleetVisualizer(agents)

    # 3. Start Vision
    print("Starting Vision...")
    if not vision.start():
        exit()

    print("Starting Experiment...")
    start_time = time.time()
    last_update = start_time

    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed > TRAJECTORY_DURATION:
                print("Trajectory finished.")
                break

            if current_time - last_update >= UPDATE_RATE:
                # A. Global Trajectory Calculation
                vc_state = vc_trajectory(elapsed)

                # =========================================================
                # PASS 1: SENSING & GLOBAL ERROR CALCULATION
                # =========================================================
                current_states = {}
                formation_errors = {} # Dict to hold GLOBAL errors

                for bot in agents:
                    # 1. Update Vision & State
                    if hasattr(vision, 'set_target_marker'):
                        vision.set_target_marker(bot.marker_id)

                    x, y, theta, detected = vision.get_robot_position()
                    current_states[bot.id] = (x, y, theta, detected)

                    if detected:
                        # 2. Reference
                        x_ref, y_ref, _, _, _ = follower_reference(vc_state, bot.offset)
                        x_ref += START_X
                        y_ref += START_Y

                        # 3. Calculate GLOBAL Error ONLY
                        ex_global = x_ref - x
                        ey_global = y_ref - y

                        # STORE GLOBAL ERROR
                        formation_errors[bot.id] = np.array([ex_global, ey_global])
                    else:
                        formation_errors[bot.id] = np.array([0.0, 0.0])

                # =========================================================
                # PASS 2: ROBOT UPDATES (CONTROL)
                # =========================================================
                for bot in agents:
                    # Pass the specific state and the FULL error dictionary
                    bot.update(
                        current_time,
                        elapsed,
                        vc_state,
                        current_states[bot.id],
                        formation_errors
                    )

                # C. Update Graphics
                viz.update()

                last_update = current_time

    except KeyboardInterrupt:
        print("\nEmergency Stop Requested.")

    finally:
        print("Shutting down...")
        vision.stop()
        for bot in agents:
            bot.disconnect()
        viz.close()