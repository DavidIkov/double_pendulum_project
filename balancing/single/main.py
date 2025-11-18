from pendulum_simulation import Pendulum, SinglePendulumSimulation
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

simulation = SinglePendulumSimulation(
    Pendulum((np.random.random()-0.5)*2*np.pi))

# Your pre-computed position data (replace with your actual data)
# Format: arrays of x and y coordinates for each dot over time
time_steps = 1000
x1_positions = []
y1_positions = []
x2_positions = []
y2_positions = []

for _ in range(time_steps):
    simulation.Step(0.05)
    x, y = simulation.GetPendulumPosition()
    x1_positions += [0]
    y1_positions += [0]
    x2_positions += [x]
    y2_positions += [y]

# Set up the figure
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(min(min(x1_positions), min(x2_positions)) - 0.5,
            max(max(x1_positions), max(x2_positions)) + 0.5)
ax.set_ylim(min(min(y1_positions), min(y2_positions)) - 0.5,
            max(max(y1_positions), max(y2_positions)) + 0.5)

# Create the dots
dot1, = ax.plot([], [], 'ro', markersize=12, label='Dot 1')
dot2, = ax.plot([], [], 'bo', markersize=12, label='Dot 2')

# Optional: Add trails to show paths
trail1, = ax.plot([], [], 'r-', alpha=0.3, linewidth=1)
trail2, = ax.plot([], [], 'b-', alpha=0.3, linewidth=1)

ax.legend()
ax.set_title('Two Dots with Pre-computed Positions')
ax.grid(True)


def init():
    dot1.set_data([], [])
    dot2.set_data([], [])
    trail1.set_data([], [])
    trail2.set_data([], [])
    return dot1, dot2, trail1, trail2


def update(frame):
    # Update current positions - FIX: Wrap single values in lists
    dot1.set_data([x1_positions[frame]], [y1_positions[frame]])
    dot2.set_data([x2_positions[frame]], [y2_positions[frame]])

    # Update trails (show path up to current frame)
    trail1.set_data(x1_positions[:frame+1], y1_positions[:frame+1])
    trail2.set_data(x2_positions[:frame+1], y2_positions[:frame+1])

    return dot1, dot2, trail1, trail2


# Create animation
anim = animation.FuncAnimation(
    fig, update, frames=time_steps,
    init_func=init, blit=True, interval=50, repeat=True
)

plt.tight_layout()
plt.show()
