
from pendulum_simulation import Pendulum, SinglePendulumSimulation
import gymnasium as gym
import numpy as np


def normalize_vec(v):
    norm = np.linalg.norm(v)
    if norm:
        return v/norm
    else:
        return v


class SinglePendulumEnv(gym.Env):
    def __init__(self):
        self.pendulum_sim_ = SinglePendulumSimulation(Pendulum())

        self.platform_acceleration_bounds = (-.2, .2)
        self.action_space = gym.spaces.Box(
            low=self.platform_acceleration_bounds[0], high=self.platform_acceleration_bounds[1], shape=(2,), dtype=np.float64)

        self.platform_position_bounds = (-50., 50.)
        self.platform_velocity_bounds = (-5, 5)
        self.angular_velocity_bounds = (-10., 10.)
        self.observation_space = gym.spaces.Box(low=np.array(
            [self.platform_position_bounds[0], self.platform_position_bounds[0], self.platform_velocity_bounds[0], self.platform_velocity_bounds[0], -1., -1., self.angular_velocity_bounds[0]]), high=np.array([self.platform_position_bounds[1], self.platform_position_bounds[1], self.platform_velocity_bounds[1], self.platform_velocity_bounds[1], 1., 1., self.angular_velocity_bounds[1]]), shape=(7,), dtype=np.float64)
        self.steps_counter = 0
        self.dt_ = 0.1

        self.platform_velocity_ = np.array([0.0, 0.0])
        self.platform_pos_ = np.array([0.0, 0.0])
        self.angular_velocity_ = 0.0

    def _get_obs(self):
        pend = self.pendulum_sim_.GetPendulum()
        return [self.platform_pos_[0], self.platform_pos_[1], self.platform_velocity_[0], self.platform_velocity_[1], np.cos(pend.angle_), np.sin(pend.angle_), self.angular_velocity_]

    def reset(self, seed=None, options=[]):
        super().reset(seed=seed)

        self.pendulum_sim_ = SinglePendulumSimulation(Pendulum((self.np_random.random(
        )-0.5)*2*np.pi/4+np.pi, self.np_random.random()*5, self.np_random.random()*5, 0))
        self.steps_counter = 0
        self.platform_velocity_ = np.array([0.0, 0.0])
        self.platform_pos_ = np.array(
            [(self.np_random.random()-0.5)*2*10, (self.np_random.random()-0.5)*2*10])
        self.angular_velocity_ = 0.0

        return self._get_obs(), {}

    def _calc_reward(self):
        reward = 0

        dot_prod = np.dot(-normalize_vec(self.platform_pos_),
                          normalize_vec(self.platform_velocity_))
        reward += dot_prod*3

        # if agent is under distance of 50, it gets reward, otherwise penalty
        # from 1 to -inf
        #distance = np.sqrt(np.sum(self.platform_pos_**2))
        #reward += 1.0 - (distance / 50.0)

        # if agent has too much velocity, it gets penalty
        # otherwise small award
        # from 0.5 to -1
        vel = np.sqrt(np.sum(self.platform_velocity_**2))
        # reward += 0.5 if vel < 4 else (4-vel)

        # if agent makes too much angular velocity, it gets penalty
        # otherwise small award
        # from 0.5 to -2
        # reward += 0.5 if self.angular_velocity_ < 5 else (
        #    5-self.angular_velocity_)/5*2

        # the more pendulum is up, the more award it gets
        # from -0.2 to 3
        reward += (np.cos(self.pendulum_sim_.GetPendulum().angle_)+1) / \
            2*(2-(-1.5))+-1.5

        return reward

    def step(self, action):
        action = np.clip(
            action, self.platform_acceleration_bounds[0], self.platform_acceleration_bounds[1])
        prev_ang = self.pendulum_sim_.GetPendulum().angle_
        self.pendulum_sim_.Step(self.dt_, (action[0]*10, action[1]*10))
        new_ang = self.pendulum_sim_.GetPendulum().angle_
        diff = new_ang-prev_ang
        if diff > np.pi:
            diff -= np.pi*2
        self.angular_velocity_ = diff
        self.platform_pos_ += self.platform_velocity_
        self.platform_velocity_ += (action[0], action[1])
        reward = self._calc_reward()
        self.steps_counter += 1
        terminated = False
        truncated = False

        if self.steps_counter > 100:
            truncated = True
        else:
            platform_position_too_far = not ((self.platform_pos_ > self.platform_position_bounds[0]) & (
                self.platform_pos_ < self.platform_position_bounds[1])).all()
            angular_velocity_too_big = not (
                self.angular_velocity_ > self.angular_velocity_bounds[0] and self.angular_velocity_ < self.angular_velocity_bounds[1])
            platform_velocity_too_big = not (
                (self.platform_velocity_ > self.platform_velocity_bounds[0]) & (self.platform_velocity_ < self.platform_velocity_bounds[1])).all()

            if platform_position_too_far or angular_velocity_too_big or platform_velocity_too_big:
                print(platform_position_too_far,
                      angular_velocity_too_big, platform_velocity_too_big)
                terminated = True
                reward = -20
        return self._get_obs(), reward, terminated, truncated, {}
