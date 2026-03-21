
import pybullet as p
import pybullet_data
import gymnasium as gym
import numpy as np
from typing import Any


class SinglePendulumEnv(gym.Env):
    def __init__(self, visualize: bool):
        self._client = p.connect(p.GUI if visualize else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -10)
        startPos = [0, 0, 1]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self._MODEL_ID = p.loadURDF(
            "model/single_pendulum.urdf", startPos, startOrientation)
        p.resetDebugVisualizerCamera(
            cameraDistance=8, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])

        self._STICK_ID = 0
        self._PENDULUM_ID = 1

        p.setJointMotorControl2(self._MODEL_ID,
                                self._STICK_ID,
                                p.VELOCITY_CONTROL,
                                force=0)

        p.setJointMotorControl2(self._MODEL_ID,
                                self._PENDULUM_ID,
                                p.VELOCITY_CONTROL,
                                force=0)

        # force
        self.action_space = gym.spaces.Box(low=-500, high=500, shape=(1,))

        # stickVelocity, pend0AngleCos, pend0AngleSin, pend0Velocity
        self.observation_space = gym.spaces.Box(
            low=np.array([-30, -1, -1, -50]),
            high=np.array([30, 1, 1, 50]), shape=(4,))

        self.max_steps = 10/(1/240)

    def _get_obs(self) -> list:
        _, stickVel, _, _ = p.getJointState(self._MODEL_ID, self._STICK_ID)
        pendulumPos, pendulumVel, _, _ = p.getJointState(
            self._MODEL_ID, self._PENDULUM_ID)
        return [
            stickVel,
            np.cos(pendulumPos),
            np.sin(pendulumPos),
            pendulumVel
        ]

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[list, dict[str, Any]]:
        super().reset(seed=seed)

        p.resetJointState(self._MODEL_ID, self._STICK_ID, 0, 0)
        p.setJointMotorControl2(self._MODEL_ID,
                                self._STICK_ID,
                                p.VELOCITY_CONTROL,
                                force=0)

        p.resetJointState(self._MODEL_ID, self._PENDULUM_ID, 0, 0)
        p.setJointMotorControl2(self._MODEL_ID,
                                self._PENDULUM_ID,
                                p.VELOCITY_CONTROL,
                                force=0)

        self.steps_counter = 0

        return self._get_obs(), {}

    def _calculate_reward(self,obs:list) -> float:
        return obs[2]

    def step(self, action: float):
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
            stick_vel_too_big = np.abs(obs[0]) > 30
            pend0_vel_too_big = np.abs(obs[3]) > 50
            if stick_vel_too_big or pend0_vel_too_big:
                print(stick_vel_too_big, pend0_vel_too_big)
                terminated = True
                reward = -10

        return obs, reward, terminated, truncated, {}
