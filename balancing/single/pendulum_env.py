
from pendulum_simulation import Pendulum, SinglePendulumSimulation
import gymnasium as gym
import numpy as np


class SinglePendulumEnv(gym.Env):
    def __init__(self):
        self.pendulum_ = SinglePendulumSimulation(Pendulum())
        self.platform_pos = (0.0, 0.0)
        self.action_space = gym.spaces.Box(
            low=-2., high=2., shape=(2,), dtype=np.float64)
        self.observation_space = gym.spaces.Box(low=np.array(
            [-5., -5.,  -1.0, -1.0]), high=np.array([5., 5.,  1.0, 1.0]), shape=(4,), dtype=np.float64)

    def _get_obs(self):
        pend = self.pendulum_.GetPendulum()
        return [self.platform_pos[0], self.platform_pos[1], np.cos(pend.angle_), np.sin(pend.angle_)]

    def reset(self, seed, options=[]):
        super().reset(seed=seed)

        self.pendulum_ = SinglePendulumSimulation(Pendulum((self.np_random.random(
        )-0.5)*2*np.pi, (self.np_random.random()-0.5)*2*1, self.np_random.random()*5, 0))

        return self._get_obs(), {}

    def step(self, action):
        self.pendulum_.Step(0.05, (action[0], action[1]))
        reward = -np.cos(self.pendulum_.angle_)
        terminated = False
        truncated = False
        return self._get_obs(), reward, terminated, truncated, {}


if __name__ == "__main__":
    aa = SinglePendulumEnv(0.0, 0.0, 0.0)
    print(aa.action_space.n)
