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

connectivity_matrix = np.array([
    [0, 1, 1], # Robot 0 listens to 1 and 2
    [1, 0, 1], # Robot 1 listens to 0 and 2
    [1, 1, 0]  # Robot 2 listens to 0 and 1
])

# Physics & Vision
WHEEL_RADIUS = 0.06471 / 2
WHEEL_BASE = 0.07782
UPDATE_RATE = 0.05   # 20 Hz
AREA_WIDTH = 0.49 * 7
AREA_HEIGHT = 0.49 * 4
CAMERA_ROI = (0, 0, 1920, 1080)
START_X, START_Y = 0, 0

# Control Gains
GAINS = {'cx': 1.1, 'ct': 1.0, 'cy': 65.0, 'consensus': 0.3}
K_PARAM = 1.0
TRAJECTORY_DURATION = 1000.0

# Fleet Config
# NOTE: The order here MUST match the rows in connectivity_matrix
ROBOT_FLEET = [
    {'id': 'RAM06', 'mac': '98:D3:32:20:28:46', 'port': '/dev/rfcomm0', 'marker_id': 7, 'offset': np.array([0.2, 0.2]), 'color': 'blue'},
    {'id': 'RAM05', 'mac': '98:D3:32:10:15:96', 'port': '/dev/rfcomm1', 'marker_id': 0, 'offset': np.array([-0.2, -0.2]), 'color': 'green'},
    {'id': 'RAM02', 'mac': '98:D3:32:30:24:38', 'port': '/dev/rfcomm2', 'marker_id': 1, 'offset': np.array([0.0, 0.0]), 'color': 'red'}
]

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def wrap_angle(angle):
    """Normalizes angle to range [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

# =============================================================================
# LOGIC CLASS: FormationRobot
# =============================================================================

class FormationRobot:
    """Handles robot connection, state estimation, and control computation."""
    def __init__(self, config, index):
        self.index = index  # My row index in connectivity matrix
        self.id = config['id']
        self.marker_id = config['marker_id']
        self.offset = config['offset']
        self.color = config['color']

        # Hardware
        self.robot = Robot(self.id, config['mac'], config['port'])
        self.controller = UnicycleController(abs(WHEEL_BASE), WHEEL_RADIUS)
        self.connected = False

        # Internal State
        self.w_cmd_filtered = 0.0
        self.w_filter_alpha = 0.3

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

    def compute_consensus_errors(self, fleet_states, vc_state):
        """
        Calculates neighbors' errors projected into THIS robot's body frame.
        """
        # 1. Get my own orientation
        my_theta = fleet_states[self.index][2]
        c_f = np.cos(my_theta)
        s_f = np.sin(my_theta)

        num_robots = len(fleet_states)
        neighbors_errors = np.zeros((num_robots, 3))

        for n_idx in range(num_robots):
            if n_idx == self.index: continue

            # A. Get Neighbor's State & Config
            x_n, y_n, theta_n = fleet_states[n_idx]
            n_offset = ROBOT_FLEET[n_idx]['offset']

            # B. Calculate Neighbor's Reference
            x_ref_n, y_ref_n, theta_ref_n, _, _ = follower_reference(vc_state, n_offset)
            x_ref_n += START_X
            y_ref_n += START_Y

            # C. Global Error
            x_e_g = x_ref_n - x_n
            y_e_g = y_ref_n - y_n
            theta_e = wrap_angle(theta_ref_n - theta_n)

            # D. Rotate to Body Frame
            x_e_l = c_f * x_e_g + s_f * y_e_g
            y_e_l = -s_f * x_e_g + c_f * y_e_g

            neighbors_errors[n_idx] = [x_e_l, y_e_l, theta_e]

        return neighbors_errors

    def update(self, current_time, elapsed_time, vc_state, my_state, fleet_states):
        """
        Calculates control based on provided state and fleet states.
        """
        if not self.connected: return

        # Unpack my state
        if my_state is None:
            # If vision lost me, stop
            self.robot.send_message(self.controller.stop_command())
            print(f"[{self.id}] Not detected")
            return

        x, y, theta = my_state

        # 1. Calculate My Reference
        x_ref, y_ref, theta_ref, theta_dot_ref, v_ref = follower_reference(vc_state, self.offset)
        x_ref += START_X
        y_ref += START_Y

        # 2. Compute Consensus Errors
        # We only do this if we have valid data for the whole fleet
        n_errors = None
        if fleet_states is not None:
            n_errors = self.compute_consensus_errors(fleet_states, vc_state)

        # 3. Compute Control
        v_cmd, w_cmd = follower_control(
            state=[x, y, theta],
            ref=(x_ref, y_ref, theta_ref, theta_dot_ref, v_ref),
            neighbors_errors=n_errors,
            connectivity_row=connectivity_matrix[self.index],
            gains=GAINS,
            k=K_PARAM
        )

        # 4. Filter & Send
        self.w_cmd_filtered = self.w_filter_alpha * w_cmd + (1 - self.w_filter_alpha) * self.w_cmd_filtered
        command = self.controller.compute_command_from_velocities(v_cmd, self.w_cmd_filtered)

        if not self.robot.send_message(command):
            print(f"[{self.id}] Comms lost, reconnecting...")
            self.robot.reconnect()

        # 5. Record Data
        err_dist = np.sqrt((x_ref - x)**2 + (y_ref - y)**2)
        err_theta = np.arctan2(np.sin(theta_ref - theta), np.cos(theta_ref - theta))
        self._record_data(elapsed_time, x, y, x_ref, y_ref, err_dist, err_theta, v_cmd, self.w_cmd_filtered)

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
    def __init__(self, robots):
        plt.ion()
        self.fig = plt.figure(figsize=(14, 9))
        self.robots = robots
        self.lines = {}

        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.5, 1])

        # Left: Trajectory
        self.ax_traj = self.fig.add_subplot(gs[:, 0])
        self.ax_traj.set_title('Formation Trajectory')
        self.ax_traj.set_xlim(-AREA_WIDTH/2, AREA_WIDTH/2)
        self.ax_traj.set_ylim(-AREA_HEIGHT/2, AREA_HEIGHT/2)
        self.ax_traj.set_aspect('equal')
        self.ax_traj.grid(True, alpha=0.3)

        # Right: Dashboard
        self.ax_dist = self.fig.add_subplot(gs[0, 1])
        self.ax_dist.set_title('Position Error (m)')
        self.ax_dist.grid(True)

        self.ax_theta = self.fig.add_subplot(gs[1, 1])
        self.ax_theta.set_title('Heading Error (deg)')
        self.ax_theta.grid(True)

        self.ax_cmd = self.fig.add_subplot(gs[2, 1])
        self.ax_cmd.set_title('Control Inputs (v: solid, w: dashed)')
        self.ax_cmd.grid(True)

        for r in robots:
            self.init_robot_lines(r)

        self.ax_traj.legend(loc='upper right')
        self.fig.tight_layout()

    def init_robot_lines(self, robot):
        trail, = self.ax_traj.plot([], [], '-', color=robot.color, linewidth=1.5, alpha=0.8, label=robot.id)
        ref, = self.ax_traj.plot([], [], '--', color=robot.color, linewidth=1, alpha=0.3)
        head, = self.ax_traj.plot([], [], 'o', color=robot.color)

        err_d, = self.ax_dist.plot([], [], '-', color=robot.color)
        err_t, = self.ax_theta.plot([], [], '-', color=robot.color)

        cmd_v, = self.ax_cmd.plot([], [], '-', color=robot.color, linewidth=1.5)
        cmd_w, = self.ax_cmd.plot([], [], '--', color=robot.color, linewidth=1.0, alpha=0.7)

        self.lines[robot.id] = {
            'trail': trail, 'ref': ref, 'head': head,
            'err_d': err_d, 'err_t': err_t,
            'cmd_v': cmd_v, 'cmd_w': cmd_w
        }

    def update(self):
        for r in self.robots:
            d = r.data
            if not d['t']: continue
            ln = self.lines[r.id]

            # Map
            ln['trail'].set_data(d['x'], d['y'])
            ln['ref'].set_data(d['ref_x'], d['ref_y'])
            ln['head'].set_data([d['x'][-1]], [d['y'][-1]])

            # Graphs
            ln['err_d'].set_data(d['t'], d['err_dist'])
            ln['err_t'].set_data(d['t'], d['err_theta'])
            ln['cmd_v'].set_data(d['t'], d['v'])
            ln['cmd_w'].set_data(d['t'], d['w'])

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
    vision = VisionTracker(4, AREA_WIDTH, AREA_HEIGHT, CAMERA_ROI)

    agents = []
    print("Initializing Fleet...")
    # Enumerate to assign index (0, 1, 2) automatically
    for i, config in enumerate(ROBOT_FLEET):
        bot = FormationRobot(config, index=i)
        if bot.connect():
            agents.append(bot)

    if not agents:
        print("No robots connected.")
        exit()

    viz = FleetVisualizer(agents)

    # 2. Start Vision
    print("Starting Vision...")
    if not vision.start():
        exit()

    # 3. Main Loop
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

                # B. SENSE: Gather ALL robot states first (Synchronous)
                current_fleet_states = []
                vision_success = True

                # Note: We must iterate through ALL potential indices 0..N
                # even if a robot failed to connect, to maintain matrix alignment.
                # However, since 'agents' only has connected bots, we map carefully.
                # Simplification: We assume 'agents' list matches fleet indices 0,1,2 for now.

                for bot in agents:
                    if hasattr(vision, 'set_target_marker'):
                        vision.set_target_marker(bot.marker_id)

                    x, y, theta, detected = vision.get_robot_position()

                    if detected:
                        current_fleet_states.append([x, y, theta])
                    else:
                        current_fleet_states.append([0,0,0]) # Placeholder
                        vision_success = False

                # C. ACT: Update All Robots
                for i, bot in enumerate(agents):
                    # Only perform consensus if vision was good for everyone
                    # (Or you can implement logic to ignore blind robots)
                    fleet_data = current_fleet_states if vision_success else None

                    my_state = None
                    if vision_success:
                        my_state = current_fleet_states[i] # This robot's state

                    bot.update(current_time, elapsed, vc_state, my_state, fleet_data)

                # D. DISPLAY: Update Graphics
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