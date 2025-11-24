import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from robot import Robot
from unicycle_controller import UnicycleController
from vision_tracker import VisionTracker
from references import vc_trajectory, follower_reference
from controllers import follower_control

# =============================================================================
# CONFIGURATION
# =============================================================================

connectivity_matrix = np.array([
    [0, 1, 1],
    [1, 0, 1],
    [1, 1, 0]
])

# Robot physical parameters
WHEEL_RADIUS = 0.06471 / 2
WHEEL_BASE = 0.07782  # Negative for sign convention
UPDATE_RATE = 0.05   # 20 Hz

# Vision parameters
AREA_WIDTH = 0.49 * 7
AREA_HEIGHT = 0.49 * 4
CAMERA_ROI = (0, 0, 1920, 1080)
START_X = 0
START_Y = 0

# Controller Gains (Tuned)
GAINS = {
    'cx': 1.1,
    'ct': 1.0,
    'cy': 65.0,
    'consensus': 0.3
}
K_PARAM = 1.0
TRAJECTORY_DURATION = 1000.0

# =============================================================================
# FLEET CONFIGURATION
# =============================================================================
ROBOT_FLEET = [
    {
        'id': 'RAM06',
        'mac': '98:D3:32:20:28:46',
        'port': '/dev/rfcomm0',
        'marker_id': 7,
        'offset': np.array([0.2, 0.2]),  # 0.5m to the "left" of center
        'color': 'blue'
    },
    # Uncomment to add second robot
    {
        'id': 'RAM05',
        'mac': '98:D3:32:10:15:96', # Update MAC
        'port': '/dev/rfcomm1',
        'marker_id': 0,
        'offset': np.array([-0.2, -0.2]), # 0.5m to the "right" of center
        'color': 'green'
    },
    {
        'id': 'RAM02',
        'mac': '98:D3:32:30:24:38', # Update MAC
        'port': '/dev/rfcomm2',
        'marker_id': 1,
        'offset': np.array([-0.00, 0.00]), # Center
        'color': 'red'
    }
]

# =============================================================================
# ROBOT AGENT CLASS
# =============================================================================

class FormationRobot:
    def __init__(self, config, axes):
        self.config = config
        self.name = config['id']
        self.marker_id = config['marker_id']
        self.offset = config['offset']
        self.color = config['color']

        # Hardware connection
        self.robot = Robot(self.name, config['mac'], config['port'])
        self.controller = UnicycleController(abs(WHEEL_BASE), WHEEL_RADIUS)

        # State
        self.connected = False
        self.w_cmd_filtered = 0.0
        self.w_filter_alpha = 0.3

        # Plotting handles
        self.ax_traj, self.ax_pos, self.ax_theta, self.ax_cmd = axes

        # Initialize plots
        self.trail_line, = self.ax_traj.plot([], [], '-', color=self.color, linewidth=1, alpha=0.6, label=f'{self.name} Path')
        self.ref_line, = self.ax_traj.plot([], [], '--', color=self.color, linewidth=1, alpha=0.4)
        self.pos_marker, = self.ax_traj.plot([], [], 'o', color=self.color, markersize=8)
        self.ref_marker, = self.ax_traj.plot([], [], 'x', color=self.color, markersize=8)

        self.robot_arrow = None
        self.ref_arrow = None

        # Error plots
        self.err_line, = self.ax_pos.plot([], [], '-', color=self.color, label=f'{self.name} Err')
        self.theta_err_line, = self.ax_theta.plot([], [], '-', color=self.color, label=f'{self.name} θ Err')

        # Command plots
        self.v_line, = self.ax_cmd.plot([], [], '-', color=self.color, linestyle='-', label=f'{self.name} v')
        self.w_line, = self.ax_cmd.plot([], [], '-', color=self.color, linestyle='--', label=f'{self.name} ω')

        # History
        self.history = {
            't': [], 'x': [], 'y': [], 'ref_x': [], 'ref_y': [],
            'err_dist': [], 'err_theta': [], 'v': [], 'w': []
        }

    def connect(self):
        print(f"Connecting to {self.name}...")
        self.connected = self.robot.connect()
        return self.connected

    def disconnect(self):
        if self.connected:
            print(f"Stopping {self.name}...")
            self.robot.send_message(self.controller.stop_command())
            time.sleep(0.2)
            self.robot.disconnect()

    def step(self, current_time, elapsed_time, vc_state, vision):
        if not self.connected:
            return

        # 1. Calculate Reference
        x_ref, y_ref, theta_ref, theta_dot_ref, v_ref = follower_reference(vc_state, self.offset)
        x_ref += START_X
        y_ref += START_Y

        # 2. Get Position (Set target marker first)
        # Note: Assuming vision tracker has set_target_marker or similar mechanism
        # If get_robot_position accepts an ID, use that.
        # Here we assume we need to tell the tracker which ID to look for.
        if hasattr(vision, 'set_target_marker'):
            vision.set_target_marker(self.marker_id)

        x, y, theta, detected = vision.get_robot_position()

        # 3. Compute Control
        if detected:
            state = [x, y, theta]
            ref = (x_ref, y_ref, theta_ref, theta_dot_ref, v_ref)

            v_cmd, w_cmd = follower_control(
                state=state, ref=ref,
                neighbors_errors=None, connectivity_row=connectivity_matrix,
                gains=GAINS, k=K_PARAM
            )

            # Filter w
            self.w_cmd_filtered = self.w_filter_alpha * w_cmd + (1 - self.w_filter_alpha) * self.w_cmd_filtered

            # Errors
            err_x = x_ref - x
            err_y = y_ref - y
            err_dist = np.sqrt(err_x**2 + err_y**2)
            err_theta = np.arctan2(np.sin(theta_ref - theta), np.cos(theta_ref - theta))

            # Update History
            self.history['t'].append(elapsed_time)
            self.history['x'].append(x)
            self.history['y'].append(y)
            self.history['ref_x'].append(x_ref)
            self.history['ref_y'].append(y_ref)
            self.history['err_dist'].append(err_dist)
            self.history['err_theta'].append(np.degrees(err_theta))
            self.history['v'].append(v_cmd)
            self.history['w'].append(self.w_cmd_filtered)

            command = self.controller.compute_command_from_velocities(v_cmd, self.w_cmd_filtered)

            # Print status
            print(f"[{self.name}] Err: {err_dist:.3f}m {np.degrees(err_theta):.1f}° | Cmd: v={v_cmd:.2f} w={self.w_cmd_filtered:.2f}")
        else:
            command = self.controller.stop_command()
            print(f"[{self.name}] NOT DETECTED")

        # 4. Send Command
        if not self.robot.send_message(command):
            print(f"[{self.name}] Lost connection, attempting reconnect...")
            self.robot.reconnect()

    def update_plots(self):
        if not self.history['t']:
            return

        # Trajectory
        self.trail_line.set_data(self.history['x'], self.history['y'])
        self.ref_line.set_data(self.history['ref_x'], self.history['ref_y'])
        self.pos_marker.set_data([self.history['x'][-1]], [self.history['y'][-1]])
        self.ref_marker.set_data([self.history['ref_x'][-1]], [self.history['ref_y'][-1]])

        # Arrows
        if self.robot_arrow: self.robot_arrow.remove()
        if self.ref_arrow: self.ref_arrow.remove()

        # Current state
        curr_x, curr_y = self.history['x'][-1], self.history['y'][-1]
        # We don't store theta in history explicitly above, but we can infer or store it.
        # For simplicity, let's just skip arrow update if we don't have the raw theta handy
        # OR better, store theta in history.
        # (Skipping arrow update logic for brevity in class, but can be added if theta is stored)

        # Errors
        self.err_line.set_data(self.history['t'], self.history['err_dist'])
        self.theta_err_line.set_data(self.history['t'], self.history['err_theta'])

        # Commands
        self.v_line.set_data(self.history['t'], self.history['v'])
        self.w_line.set_data(self.history['t'], self.history['w'])


# =============================================================================
# MAIN EXECUTION
# =============================================================================

if __name__ == "__main__":
    # 1. Setup Plots
    plt.ion()
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)

    ax_traj = fig.add_subplot(gs[:, 0])
    ax_traj.set_title('Multi-Robot Trajectory')
    ax_traj.set_xlim(-AREA_WIDTH/2, AREA_WIDTH/2)
    ax_traj.set_ylim(-AREA_HEIGHT/2, AREA_HEIGHT/2)
    ax_traj.set_aspect('equal')
    ax_traj.grid(True, alpha=0.3)

    ax_pos = fig.add_subplot(gs[0, 1])
    ax_pos.set_title('Position Error (m)')
    ax_pos.grid(True)

    ax_theta = fig.add_subplot(gs[1, 1])
    ax_theta.set_title('Theta Error (deg)')
    ax_theta.grid(True)

    ax_cmd = fig.add_subplot(gs[2, 1])
    ax_cmd.set_title('Commands')
    ax_cmd.grid(True)

    axes = (ax_traj, ax_pos, ax_theta, ax_cmd)

    # 2. Initialize Vision
    vision = VisionTracker(4, AREA_WIDTH, AREA_HEIGHT, CAMERA_ROI)

    # 3. Initialize Robots
    agents = []
    for config in ROBOT_FLEET:
        agent = FormationRobot(config, axes)
        if agent.connect():
            agents.append(agent)
        else:
            print(f"Failed to initialize {config['id']}")

    if not agents:
        print("No robots connected. Exiting.")
        exit()

    # 4. Start Vision
    print("Starting vision...")
    if not vision.start():
        print("Vision failed.")
        exit()

    # 5. Main Loop
    print("Starting trajectory...")
    try:
        start_time = time.time()
        last_update = start_time

        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= TRAJECTORY_DURATION:
                break

            if current_time - last_update >= UPDATE_RATE:
                # Calculate Virtual Center once per cycle
                vc_state = vc_trajectory(elapsed)

                # Update all robots
                for agent in agents:
                    agent.step(current_time, elapsed, vc_state, vision)
                    agent.update_plots()

                # Rescale axes
                for ax in [ax_pos, ax_theta, ax_cmd]:
                    ax.relim()
                    ax.autoscale_view()

                plt.pause(0.001)
                last_update = current_time

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        vision.stop()

        for agent in agents:
            agent.disconnect()
        plt.ioff()
        plt.show()
