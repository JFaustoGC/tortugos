"""Animation utilities for visualizing the simulation."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class SimulationAnimator:
    """Handles animation of virtual center and followers."""
    
    def __init__(self, virtual_center_traj, followers_states, followers_ref_trajs, 
                 followers_errors, t_eval, dt):
        """
        Initialize animator.
        
        Args:
            virtual_center_traj: (N, 3) array of VC trajectory
            followers_states: (num_followers, N, 3) array of follower states
            followers_ref_trajs: (num_followers, N, 3) array of follower references
            followers_errors: (num_followers, N, 3) array of tracking errors
            t_eval: time vector
            dt: time step
        """
        self.virtual_center_traj = virtual_center_traj
        self.followers_states = followers_states
        self.followers_ref_trajs = followers_ref_trajs
        self.followers_errors = followers_errors
        self.t_eval = t_eval
        self.dt = dt
        self.num_followers = followers_states.shape[0]
        self.N = len(t_eval)
        
        self._setup_figure()
    
    def _setup_figure(self):
        """Setup figure and axes."""
        self.fig, (self.ax_main, self.ax_err) = plt.subplots(
            2, 1, figsize=(10, 11), gridspec_kw={'height_ratios': [3, 1]}
        )
        
        # Main trajectory plot
        self.ax_main.set_aspect('equal', 'box')
        self.ax_main.set_xlabel('X (m)')
        self.ax_main.set_ylabel('Y (m)')
        self.ax_main.set_xlim(-2, 2)
        self.ax_main.set_ylim(-2, 2)
        self.ax_main.grid(True, linestyle='--', alpha=0.3)
        self.ax_main.set_title("Virtual Center and Followers Trajectories")
        
        # VC visualization
        self.vc_path_line, = self.ax_main.plot([], [], 'r--', lw=2, label='VC Path (Ref)')
        self.vc_marker, = self.ax_main.plot([], [], 'ro', markersize=6)
        self.vc_arrow = self.ax_main.quiver([], [], [], [], color='r', units='xy', scale=0.6)
        
        # Followers visualization
        colors = ['m', 'g', 'c', 'y', 'orange', 'purple']
        self.f_paths, self.f_markers, self.f_arrows, self.f_refs = [], [], [], []
        for f_idx, color in enumerate(colors[:self.num_followers]):
            path_line, = self.ax_main.plot([], [], color + '-', lw=1.8, 
                                          label=f'Follower {f_idx + 1}')
            marker_line, = self.ax_main.plot([], [], color + 'o', markersize=5)
            arrow = self.ax_main.quiver([], [], [], [], color=color, units='xy', scale=0.6)
            ref_line, = self.ax_main.plot([], [], color + '--', alpha=0.4, lw=1, 
                                         label=f'Follower {f_idx + 1} Ref')
            self.f_paths.append(path_line)
            self.f_markers.append(marker_line)
            self.f_arrows.append(arrow)
            self.f_refs.append(ref_line)
        
        self.ax_main.legend(loc='upper right', fontsize=9)
        
        # Error plot
        self.ax_err.set_xlim(0, self.t_eval[-1])
        self.ax_err.set_ylim(-0.5, 0.5)
        self.ax_err.set_xlabel('Time (s)')
        self.ax_err.set_ylabel('Tracking Error')
        self.ax_err.set_title('Follower Tracking Errors')
        self.ax_err.grid(True, linestyle='--', alpha=0.4)
        
        self.err_lines = []
        for f_idx, color in enumerate(colors[:self.num_followers]):
            line_x, = self.ax_err.plot([], [], color + '-', lw=1.2, 
                                      label=f'Follower {f_idx + 1} x_e')
            line_y, = self.ax_err.plot([], [], color + '--', lw=1.2, 
                                      label=f'Follower {f_idx + 1} y_e')
            line_th, = self.ax_err.plot([], [], color + ':', lw=1.2, 
                                       label=f'Follower {f_idx + 1} θ_e')
            self.err_lines.append((line_x, line_y, line_th))
        
        self.ax_err.legend(ncol=2, fontsize=8, loc='upper right')
    
    def _init_anim(self):
        """Initialize animation."""
        self.vc_path_line.set_data([], [])
        self.vc_marker.set_data([], [])
        self.vc_arrow.set_offsets(np.empty((0, 2)))
        self.vc_arrow.set_UVC(np.empty(0), np.empty(0))
        
        for f_idx in range(self.num_followers):
            self.f_paths[f_idx].set_data([], [])
            self.f_refs[f_idx].set_data([], [])
            self.f_markers[f_idx].set_data([], [])
            self.f_arrows[f_idx].set_offsets(np.empty((0, 2)))
            self.f_arrows[f_idx].set_UVC(np.empty(0), np.empty(0))
            for line in self.err_lines[f_idx]:
                line.set_data([], [])
        
        return (
            self.vc_path_line, self.vc_marker, self.vc_arrow,
            *self.f_paths, *self.f_refs, *self.f_markers, *self.f_arrows, 
            *sum(self.err_lines, ())
        )
    
    def _animate(self, i):
        """Animation function."""
        # VC animation
        self.vc_path_line.set_data(self.virtual_center_traj[:i + 1, 0], 
                                   self.virtual_center_traj[:i + 1, 1])
        self.vc_marker.set_data([self.virtual_center_traj[i, 0]], 
                               [self.virtual_center_traj[i, 1]])
        self.vc_arrow.set_offsets([self.virtual_center_traj[i, 0], 
                                   self.virtual_center_traj[i, 1]])
        self.vc_arrow.set_UVC(0.5 * np.cos(self.virtual_center_traj[i, 2]),
                             0.5 * np.sin(self.virtual_center_traj[i, 2]))

        # Followers animation
        for f_idx in range(self.num_followers):
            f_state = self.followers_states[f_idx]
            f_ref = self.followers_ref_trajs[f_idx]

            x_f, y_f, th_f = f_state[i]

            # Actual path
            self.f_paths[f_idx].set_data(f_state[:i + 1, 0], f_state[:i + 1, 1])
            self.f_markers[f_idx].set_data([x_f], [y_f])
            self.f_arrows[f_idx].set_offsets([x_f, y_f])
            self.f_arrows[f_idx].set_UVC(0.4 * np.cos(th_f), 0.4 * np.sin(th_f))

            # Reference path
            self.f_refs[f_idx].set_data(f_ref[:i + 1, 0], f_ref[:i + 1, 1])

            # Errors
            t_slice = self.t_eval[:i + 1]
            err = self.followers_errors[f_idx, :i + 1]
            for j, line in enumerate(self.err_lines[f_idx]):
                line.set_data(t_slice, err[:, j])

        return (
            self.vc_path_line, self.vc_marker, self.vc_arrow,
            *self.f_paths, *self.f_refs, *self.f_markers, *self.f_arrows, 
            *sum(self.err_lines, ())
        )
    
    def animate(self):
        """Create and return animation."""
        anim = FuncAnimation(self.fig, self._animate, init_func=self._init_anim,
                           frames=self.N, interval=self.dt * 1000, blit=True)
        plt.tight_layout()
        return anim
    
    def show(self):
        """Show the animation."""
        self.anim = self.animate()  # Store reference to prevent garbage collection
        plt.show()

    def save(self, filename, writer=None):
        """Save the animation to a file.
        
        Args:
            filename: Name of the output file (e.g., 'animation.gif').
            writer: Optional writer instance for saving (e.g., PillowWriter).
        """
        self.anim = self.animate()  # Store reference to prevent garbage collection
        self.anim.save(filename, writer=writer)
