import numpy as np
import torch

class ExperienceReplay:
    """
    Standard Replay Buffer for Off-Policy DRL Algorithms (DDPG/SAC/TD3).
    Stores transitions: (State, Action, Reward, Next_State, Done).
    """
    def __init__(self, max_size=50000, state_dim=38, action_dim=2):
        self.max_size = max_size
        self.ptr = 0
        self.size = 0

        # Pre-allocate memory for speed (Crucial for CPU optimization)
        self.state = np.zeros((max_size, state_dim), dtype=np.float32)
        self.action = np.zeros((max_size, action_dim), dtype=np.float32)
        self.reward = np.zeros((max_size, 1), dtype=np.float32)
        self.next_state = np.zeros((max_size, state_dim), dtype=np.float32)
        self.done = np.zeros((max_size, 1), dtype=np.float32)

    def add(self, state, action, reward, next_state, done):
        """Saves a transition into the buffer."""
        self.state[self.ptr] = state
        self.action[self.ptr] = action
        self.reward[self.ptr] = reward
        self.next_state[self.ptr] = next_state
        self.done[self.ptr] = 1.0 if done else 0.0

        # Circular buffer logic
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)

    def sample(self, batch_size=64):
        """Randomly samples a batch of experiences for training."""
        ind = np.random.randint(0, self.size, size=batch_size)
        # np.ascontiguousarray ensures a single contiguous copy (required for torch.as_tensor);
        # torch.as_tensor then wraps it with zero extra copies (arrays are already float32).
        return (
            torch.as_tensor(np.ascontiguousarray(self.state[ind])),
            torch.as_tensor(np.ascontiguousarray(self.action[ind])),
            torch.as_tensor(np.ascontiguousarray(self.reward[ind])),
            torch.as_tensor(np.ascontiguousarray(self.next_state[ind])),
            torch.as_tensor(np.ascontiguousarray(self.done[ind])),
        )