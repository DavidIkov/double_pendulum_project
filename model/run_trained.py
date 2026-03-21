import numpy as np
import time
import keras
from single_pendulum_env import SinglePendulumEnv

# Create the environment (render=True to see the simulation)
env = SinglePendulumEnv(True)

# Get environment parameters (same as in training)
num_states = env.observation_space.shape[0]
num_actions = env.action_space.shape[0]
upper_bound = env.action_space.high[0]
lower_bound = env.action_space.low[0]

# Load the trained actor model
actor_model = keras.models.load_model('pendulum_actor3.keras')
print("Actor model loaded.")

def get_action(state):
    """
    Returns the action for a given state using the trained actor.
    """
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

print("Running trained model...")
while True:
    action = get_action(state)
    state, reward, done, truncated, _ = env.step(action)
    print(reward,end="\r")
    total_reward += reward
    step += 1
    if done or truncated:
        env.reset()
    time.sleep(env.simulation_dt)
