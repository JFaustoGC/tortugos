"""Main simulation orchestration."""
import numpy as np
from utils import wrap_angle, unicycle_step
from references import vc_trajectory, follower_reference
from controllers import follower_control


class FormationSimulator:
    """Orchestrates the formation control simulation."""
    
    def __init__(self, T=50.0, dt=0.05, followers_offsets=None, 
                 connectivity_matrix=None, gains=None, k=0):
        """
        Initialize simulator.
        
        Args:
            T: simulation time
            dt: time step
            followers_offsets: list of np.array([Px, Py]) offsets
            connectivity_matrix: (num_followers, num_followers) connectivity
            gains: dict with 'cx', 'ct', 'cy', 'consensus' keys
            k: design parameter
        """
        self.T = T
        self.dt = dt
        self.N = int(T / dt) + 1
        self.t_eval = np.linspace(0, T, self.N)
        
        # Followers configuration
        if followers_offsets is None:
            followers_offsets = [
                np.array([0.5, 0.5]),
                np.array([-0.5, -0.5]),
                np.array([0.5, -0.5])
            ]
        self.followers_P = followers_offsets
        self.num_followers = len(followers_offsets)
        
        # Connectivity
        if connectivity_matrix is None:
            connectivity_matrix = np.eye(self.num_followers)
        self.connectivity_matrix = connectivity_matrix
        
        # Controller parameters
        if gains is None:
            gains = {'cx': 0.8, 'ct': 10.0, 'cy': 1.2, 'consensus': 0.0}
        self.gains = gains
        self.k = k
        
        # Storage
        self.virtual_center_traj = np.zeros((self.N, 3))
        self.followers_states = np.zeros((self.num_followers, self.N, 3))
        self.followers_errors = np.zeros((self.num_followers, self.N, 3))
        self.followers_ref_trajs = np.zeros((self.num_followers, self.N, 3))
        
        self._initialized = False
    
    def initialize(self):
        """Initialize robot positions."""
        x0, y0, theta0, *_ = vc_trajectory(0)
        self.virtual_center_traj[0] = [x0, y0, theta0]
        
        for f_idx, P in enumerate(self.followers_P):
            x_f0 = x0 + P[0] * np.cos(theta0) - P[1] * np.sin(theta0)
            y_f0 = y0 + P[0] * np.sin(theta0) + P[1] * np.cos(theta0)
            self.followers_states[f_idx, 0] = [x_f0, y_f0, theta0]
        
        self._initialized = True
    
    def run(self, perturbation_func=None):
        """
        Run the simulation.
        
        Args:
            perturbation_func: callable(t, f_idx) -> (v, w) or None
                               Returns control override if applicable
        
        Returns:
            dict with 'vc_traj', 'followers_states', 'followers_errors', 
            'followers_refs', 't_eval'
        """
        if not self._initialized:
            self.initialize()
        
        # Precompute reference trajectories
        for i, t in enumerate(self.t_eval):
            vc_state = vc_trajectory(t)
            for f_idx, P in enumerate(self.followers_P):
                x_r, y_r, th_r, *_ = follower_reference(vc_state, P)
                self.followers_ref_trajs[f_idx, i, :] = [x_r, y_r, th_r]
        
        # Main simulation loop
        for i, t in enumerate(self.t_eval[1:], start=1):
            # Compute VC state
            vc_state = vc_trajectory(t)
            self.virtual_center_traj[i] = vc_state[:3]
            
            for f_idx, P in enumerate(self.followers_P):
                # Get reference
                f_ref = follower_reference(vc_state, P)
                
                # Compute neighbors' errors in this follower's body frame
                neighbors_errors = self._compute_neighbors_errors(f_idx, i, vc_state)
                
                # Get connectivity
                connectivity_row = self.connectivity_matrix[f_idx]
                
                # Compute control
                v_f, w_f = follower_control(
                    self.followers_states[f_idx, i - 1], 
                    f_ref,
                    neighbors_errors=neighbors_errors,
                    connectivity_row=connectivity_row,
                    gains=self.gains,
                    k=self.k
                )
                
                # Apply perturbation if provided
                if perturbation_func is not None:
                    override = perturbation_func(t, f_idx)
                    if override is not None:
                        v_f, w_f = override
                
                # Integrate
                self.followers_states[f_idx, i] = unicycle_step(
                    self.followers_states[f_idx, i - 1], v_f, w_f, self.dt
                )
                
                # Compute tracking error
                x_e = f_ref[0] - self.followers_states[f_idx, i, 0]
                y_e = f_ref[1] - self.followers_states[f_idx, i, 1]
                theta_e = wrap_angle(f_ref[2] - self.followers_states[f_idx, i, 2])
                self.followers_errors[f_idx, i] = [x_e, y_e, theta_e]
        
        return {
            'vc_traj': self.virtual_center_traj,
            'followers_states': self.followers_states,
            'followers_errors': self.followers_errors,
            'followers_refs': self.followers_ref_trajs,
            't_eval': self.t_eval
        }
    
    def _compute_neighbors_errors(self, f_idx, time_idx, vc_state):
        """
        Compute each neighbor's error in their own frame, 
        then transform to current follower's frame for consensus.
        """
        neighbors_states = self.followers_states[:, time_idx - 1, :]
        neighbors_errors = np.zeros_like(neighbors_states)
        
        # Get current follower's orientation for transformations
        theta_follower = self.followers_states[f_idx, time_idx - 1, 2]
        c_f = np.cos(theta_follower)
        s_f = np.sin(theta_follower)
        
        for n_idx in range(self.num_followers):
            # Neighbor global state
            x_n, y_n, theta_n = neighbors_states[n_idx]
            
            # Neighbor reference
            x_ref_n, y_ref_n, theta_ref_n, *_ = follower_reference(
                vc_state, self.followers_P[n_idx]
            )
            
            # Global-frame error
            x_e_g = x_ref_n - x_n
            y_e_g = y_ref_n - y_n
            theta_e = wrap_angle(theta_ref_n - theta_n)
            
            # Convert to current follower's body frame for consensus
            x_e_l = c_f * x_e_g + s_f * y_e_g
            y_e_l = -s_f * x_e_g + c_f * y_e_g
            
            neighbors_errors[n_idx] = [x_e_l, y_e_l, theta_e]
        
        return neighbors_errors
