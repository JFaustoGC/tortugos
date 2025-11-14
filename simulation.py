"""
Formation control simulation - Main orchestration script.

This script sets up and runs a multi-robot formation control simulation
with consensus-based control and visualization.
"""
import numpy as np
from simulator import FormationSimulator
from animator import SimulationAnimator


# --- Simulation Configuration ---
T = 50.0
dt = 0.05

# Followers offsets in body frame
followers_offsets = [
    np.array([0.5, 0.5]),
    np.array([-0.5, -0.5]),
    np.array([0.5, -0.5])
]

# Connectivity matrix (rows = follower, columns = others it receives from)
connectivity_matrix = np.array([
    [0, 1, 1],
    [1, 0, 1],
    [1, 1, 0]
])
# connectivity_matrix = np.eye(len(followers_offsets))  # No communication

# Controller gains
gains = {
    'cx': 0.8,
    'ct': 10.0,
    'cy': 1.2,
    'consensus': 0.3  # Consensus weight (set to 0.3 to see effect of perturbation)
}

k = 0  # Design parameter for alpha


# --- Perturbation function ---
def perturbation(t, f_idx):
    """Apply perturbation to follower 0 between t=10 and t=20."""
    if 10.0 < t < 20.0 and f_idx == 0:
        return (0.0, 0.0)  # Stop follower 0
    return None  # No override


# --- Run Simulation ---
if __name__ == "__main__":
    # Create simulator
    simulator = FormationSimulator(
        T=T,
        dt=dt,
        followers_offsets=followers_offsets,
        connectivity_matrix=connectivity_matrix,
        gains=gains,
        k=k
    )
    
    # Run simulation
    print("Running simulation...")
    results = simulator.run(perturbation_func=perturbation)
    print(f"Simulation complete. Generated {len(results['t_eval'])} timesteps.")
    
    # Create animation
    print("Creating animation...")
    animator = SimulationAnimator(
        virtual_center_traj=results['vc_traj'],
        followers_states=results['followers_states'],
        followers_ref_trajs=results['followers_refs'],
        followers_errors=results['followers_errors'],
        t_eval=results['t_eval'],
        dt=dt
    )
    
    # Show animation
    print("Displaying animation... (this may take a moment)")
    animator.show()
    print("Animation window closed.")

