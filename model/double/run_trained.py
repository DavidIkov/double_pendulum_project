import numpy as np
import time
import os
os.environ["KERAS_BACKEND"] = "tensorflow"
import keras
from pendulum_env import DoublePendulumEnv

# Create the environment (render=True to see the simulation)
env = DoublePendulumEnv(True)

# Get environment parameters (same as in training)
num_states = env.observation_space.shape[0]
num_actions = env.action_space.shape[0]
upper_bound = env.action_space.high[0]
lower_bound = env.action_space.low[0]

# Load the trained actor model
actor_model = keras.models.load_model('double_pendulum_actor.keras')


def get_action(state):
    # Convert state to tensor and add batch dimension
    state_tensor = keras.ops.expand_dims(
        keras.ops.convert_to_tensor(state, dtype='float32'), 0
    )
    # Get action from actor (training=False to avoid gradient tracking)
    action = actor_model(state_tensor, training=False)
    # Clip to environment's action bounds (safety, though the model already respects them)
    action = np.clip(action.numpy().squeeze(), lower_bound, upper_bound)
    return [float(action)]  # Environment expects a list with one element


# Run an episode with the trained policy
state, _ = env.reset()
done = False
truncated = False
total_reward = 0
step = 0

while True:
    action = get_action(state)
    state, reward, done, truncated, _ = env.step(action)
    print(reward, end="\r")
    total_reward += reward
    step += 1
    if done:
        env.reset()
    time.sleep(env.simulation_dt)
