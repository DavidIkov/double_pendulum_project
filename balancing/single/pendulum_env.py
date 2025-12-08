
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

        self.platform_acceleration_bounds = (-1, 1)
        self.action_space = gym.spaces.Box(
            low=self.platform_acceleration_bounds[0], high=self.platform_acceleration_bounds[1], shape=(1,), dtype=np.float64)

        self.platform_position_bounds = (-50., 50.)
        self.platform_velocity_bounds = (-5, 5)
        self.angular_velocity_bounds = (-2., 2.)
        self.observation_space = gym.spaces.Box(low=np.array(
            [self.platform_position_bounds[0], self.platform_position_bounds[0], self.platform_velocity_bounds[0], self.platform_velocity_bounds[0], -1., -1., self.angular_velocity_bounds[0]]), high=np.array([self.platform_position_bounds[1], self.platform_position_bounds[1], self.platform_velocity_bounds[1], self.platform_velocity_bounds[1], 1., 1., self.angular_velocity_bounds[1]]), shape=(7,), dtype=np.float64)
        self.steps_counter = 0
        self.dt_ = 0.1

        self.platform_velocity_ = np.array([0.0, 0.0])
        self.platform_pos_ = np.array([0.0, 0.0])
        self.angular_velocity_ = 0.0
        self.fell_over_counter = 0

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
        self.fell_over_counter = 0

        return self._get_obs(), {}

    def _calc_reward(self):
        reward = 0
        vel = np.sqrt(np.sum(self.platform_velocity_**2))
        #reward = reward-np.abs(vel)+1

        ang_vel=np.abs(self.angular_velocity_)
        if ang_vel<0.1:
            reward+=ang_vel/0.1*2
        else:
            reward+=(0.1-ang_vel)*10

        dist=np.abs(self.platform_pos_[0])
        if dist>5:
            reward-=dist-5
        else:
            reward+=1-dist/5

        angle_cos=np.cos(self.pendulum_sim_.GetPendulum().angle_)
        if angle_cos>-0.8:
            reward-=(angle_cos--0.8)/(1--0.8)*2
        else:
            reward+=(-angle_cos-0.8)/(1-0.8)*3

        return reward

    def step(self, action):
        action[0]=10
        if action[0]<self.platform_acceleration_bounds[0] or action[0]>self.platform_acceleration_bounds[1]:
            print("wow..")
        action = np.clip(
            action, self.platform_acceleration_bounds[0], self.platform_acceleration_bounds[1])
        prev_ang = self.pendulum_sim_.GetPendulum().angle_
        self.pendulum_sim_.Step(self.dt_, (action[0], 0))
        new_ang = self.pendulum_sim_.GetPendulum().angle_
        diff = new_ang-prev_ang
        if diff > np.pi:
            diff -= np.pi*2
        self.angular_velocity_ = diff
        if np.cos(prev_ang) <= 0 and np.cos(new_ang) > 0:
            self.fell_over_counter += 1
        self.platform_pos_ += self.platform_velocity_
        self.platform_velocity_ += (action[0]*self.dt_, 0)
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
            fell_over_too_much = self.fell_over_counter > 2

            if platform_position_too_far or angular_velocity_too_big or platform_velocity_too_big or fell_over_too_much:
                print(platform_position_too_far,
                      angular_velocity_too_big, platform_velocity_too_big, fell_over_too_much)
                terminated = True
                reward = -20
        return self._get_obs(), reward, terminated, truncated, {}
