
import time
from pendulum_env import SinglePendulumEnv

env=SinglePendulumEnv(True)

env.reset()

while True:

    _,reward,truncated,terminated,_=env.step(0)
    print(reward)
    if truncated or terminated:
        env.reset()
