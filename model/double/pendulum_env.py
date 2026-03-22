
import pybullet as p
import pybullet_data
import gymnasium as gym
import numpy as np
from typing import Any


class DoublePendulumEnv(gym.Env):
    def __init__(self, visualize: bool):
        self._client = p.connect(p.GUI if visualize else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -10)
        startPos = [0, 0, 1]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self._MODEL_ID = p.loadURDF(
            "model/double/pendulum.urdf", startPos, startOrientation)
        p.resetDebugVisualizerCamera(
            cameraDistance=8, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])

        self._STICK_ID = 0
        self._PENDULUM0_ID = 1
        self._PENDULUM1_ID = 2

        # force
        self.action_space = gym.spaces.Box(low=-500, high=500, shape=(1,))

        # stickVelocity, pend0AngleCos, pend0AngleSin, pend0Velocity, pend1AngleCos, pend1AngleSin, pend1Velocity
        self.observation_space = gym.spaces.Box(
            low=np.array([-30, -1, -1, -50, -1, -1, -50]),
            high=np.array([30, 1, 1, 50, 1, 1, 50]), shape=(7,))

        self.simulation_dt = 1/60
        # 10 seconds of simulation
        self.max_steps = 10/self.simulation_dt
        p.setTimeStep(self.simulation_dt)

    def _get_obs(self) -> list:
        _, stickVel, _, _ = p.getJointState(self._MODEL_ID, self._STICK_ID)
        pendulum0Pos, pendulum0Vel, _, _ = p.getJointState(
            self._MODEL_ID, self._PENDULUM0_ID)
        pendulum1Pos, pendulum1Vel, _, _ = p.getJointState(
            self._MODEL_ID, self._PENDULUM1_ID)

        return [
            stickVel,
            np.cos(pendulum0Pos),
            np.sin(pendulum0Pos),
            pendulum0Vel,
            np.cos(pendulum1Pos),
            np.sin(pendulum1Pos),
            pendulum1Vel
        ]

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[list, dict[str, Any]]:
        super().reset(seed=seed)

        stick_angle = self.np_random.uniform(-np.pi, np.pi)
        stick_vel = self.np_random.uniform(-2.0, 2.0)
        p.resetJointState(self._MODEL_ID, self._STICK_ID,
                          stick_angle, stick_vel)
        p.setJointMotorControl2(self._MODEL_ID,
                                self._STICK_ID,
                                p.VELOCITY_CONTROL,
                                force=0)

        pend0_angle = self.np_random.uniform(-np.pi, np.pi)
        pend0_vel = self.np_random.uniform(-5.0, 5.0)
        p.resetJointState(self._MODEL_ID, self._PENDULUM0_ID,
                          pend0_angle, pend0_vel)
        p.setJointMotorControl2(self._MODEL_ID,
                                self._PENDULUM0_ID,
                                p.VELOCITY_CONTROL,
                                force=0)

        pend1_angle = self.np_random.uniform(-np.pi, np.pi)
        pend1_vel = self.np_random.uniform(-5.0, 5.0)
        p.resetJointState(self._MODEL_ID, self._PENDULUM1_ID,
                          pend1_angle, pend1_vel)
        p.setJointMotorControl2(self._MODEL_ID,
                                self._PENDULUM1_ID,
                                p.VELOCITY_CONTROL,
                                force=0)

        self.steps_counter = 0

        return self._get_obs(), {}

    def _calculate_reward(self, obs: list) -> float:
        # sin(a0+a1)=sin(a0)*cos(a1)+cos(a0)*sin(a1)
        second_angle = obs[2]*obs[4]+obs[1]*obs[5]
        print(" "*20, "\r", f"{obs[2]:.2f}, {second_angle:.2f}", end="")
        return obs[2]*2+second_angle  # -obs[0]**2/5

    def step(self, action: list):

        # clamp action
        action = np.clip(
            action[0], self.action_space.low[0], self.action_space.high[0])

        p.setJointMotorControl2(bodyUniqueId=self._MODEL_ID,
                                jointIndex=self._STICK_ID,
                                controlMode=p.TORQUE_CONTROL,
                                force=action)

        p.stepSimulation()

        self.steps_counter += 1

        obs = self._get_obs()

        truncated = False
        terminated = False
        reward = self._calculate_reward(obs)

        if self.steps_counter > self.max_steps:
            truncated = True
        else:
            stick_vel_too_big = np.abs(obs[0]) > self.observation_space.high[0]
            pend0_vel_too_big = np.abs(obs[3]) > self.observation_space.high[3]
            pend1_vel_too_big = np.abs(obs[6]) > self.observation_space.high[6]
            if stick_vel_too_big or pend0_vel_too_big or pend1_vel_too_big:
                print(stick_vel_too_big, pend0_vel_too_big, pend1_vel_too_big)
                terminated = True
                reward = -10

        return obs, reward, terminated, truncated, {}

    def close(self):
        self._client.disconnect()
