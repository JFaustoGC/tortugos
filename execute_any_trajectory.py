"""
Example script showing how to execute different trajectories on real robots.

Usage:
    python execute_any_trajectory.py vc           # Execute VC trajectory
    python execute_any_trajectory.py follower 0   # Execute follower 0 trajectory
    python execute_any_trajectory.py follower 1   # Execute follower 1 trajectory
"""
import sys
import numpy as np
from trajectory_executor import TrajectoryExecutor
from references import vc_trajectory, follower_reference


# Robot configurations
ROBOTS = {
    'ram06': {
        'mac': '98:D3:32:20:28:46',
        'port': '/dev/rfcomm0'
    },
    'ram05': {
        'mac': '98:D3:32:10:15:96',
        'port': '/dev/rfcomm1'
    }
}

# Physical parameters
WHEEL_RADIUS = 0.06471 / 2  # meters
WHEEL_BASE = 0.07782         # meters

# Follower offsets (same as simulation)
FOLLOWER_OFFSETS = [
    np.array([0.5, 0.5]),
    np.array([-0.5, -0.5]),
    np.array([0.5, -0.5])
]


def create_follower_trajectory(follower_idx):
    """
    Create a trajectory function for a specific follower.
    
    Args:
        follower_idx: Index of the follower (0, 1, 2, ...)
    
    Returns:
        trajectory_func: Function that takes time t and returns trajectory data
    """
    P = FOLLOWER_OFFSETS[follower_idx]
    
    def follower_traj(t):
        """Follower trajectory at time t."""
        vc_state = vc_trajectory(t)
        return follower_reference(vc_state, P)
    
    return follower_traj


def main():
    """Main execution function."""
    
    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python execute_any_trajectory.py vc                  # Execute VC trajectory")
        print("  python execute_any_trajectory.py follower <index>    # Execute follower trajectory")
        print("\nExample:")
        print("  python execute_any_trajectory.py vc")
        print("  python execute_any_trajectory.py follower 0")
        sys.exit(1)
    
    trajectory_type = sys.argv[1].lower()
    
    # Determine which trajectory to execute
    if trajectory_type == 'vc':
        traj_func = vc_trajectory
        traj_name = "Virtual Center"
    elif trajectory_type == 'follower':
        if len(sys.argv) < 3:
            print("Error: Please specify follower index (0, 1, 2, ...)")
            sys.exit(1)
        
        follower_idx = int(sys.argv[2])
        if follower_idx < 0 or follower_idx >= len(FOLLOWER_OFFSETS):
            print(f"Error: Follower index must be between 0 and {len(FOLLOWER_OFFSETS)-1}")
            sys.exit(1)
        
        traj_func = create_follower_trajectory(follower_idx)
        traj_name = f"Follower {follower_idx}"
    else:
        print(f"Error: Unknown trajectory type '{trajectory_type}'")
        print("Valid types: vc, follower")
        sys.exit(1)
    
    # Configuration
    ROBOT_NAME = 'ram06'  # Change this to use different robot
    DURATION = 20.0       # seconds
    UPDATE_RATE = 0.05    # 20 Hz
    
    robot_config = ROBOTS[ROBOT_NAME]
    
    print(f"\n{'='*60}")
    print(f"Trajectory Executor")
    print(f"{'='*60}")
    print(f"Robot: {ROBOT_NAME.upper()}")
    print(f"Trajectory: {traj_name}")
    print(f"Duration: {DURATION}s")
    print(f"Update Rate: {1/UPDATE_RATE:.0f} Hz")
    print(f"{'='*60}\n")
    
    # Create executor
    executor = TrajectoryExecutor(
        robot_name=ROBOT_NAME.upper(),
        mac_address=robot_config['mac'],
        rfcomm_port=robot_config['port'],
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE
    )
    
    # Connect and execute
    print(f"Connecting to {ROBOT_NAME.upper()}...")
    if executor.connect():
        print("✓ Connected successfully!\n")
        
        print(f"Executing {traj_name} trajectory...")
        print("Press Ctrl+C to stop\n")
        
        success = executor.execute_trajectory(
            trajectory_func=traj_func,
            duration=DURATION,
            dt=UPDATE_RATE,
            verbose=True
        )
        
        if success:
            print("\n✓ Trajectory execution completed successfully!")
        else:
            print("\n✗ Trajectory execution failed or was interrupted.")
        
        executor.disconnect()
    else:
        print(f"\n✗ Failed to connect to {ROBOT_NAME.upper()}")
        print("\nTroubleshooting:")
        print(f"1. Check Bluetooth pairing: {robot_config['mac']}")
        print(f"2. Bind RFCOMM device:")
        print(f"   sudo rfcomm bind {robot_config['port']} {robot_config['mac']}")
        print(f"3. Check device permissions:")
        print(f"   sudo chmod 666 {robot_config['port']}")
        sys.exit(1)


if __name__ == "__main__":
    main()
