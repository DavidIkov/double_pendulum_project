import supress_log_dump
from pendulum_env import SinglePendulumEnv
import tensorflow as tf
import keras
import numpy as np
import pygame as pg
import time


seed = 42
env = SinglePendulumEnv()
env.reset(seed=seed)

loaded_model = keras.models.load_model('trained_model.keras')

state, _ = env.reset()

pg.init()

font = pg.font.SysFont('Noto Mono', 30)

window = pg.display.set_mode((800, 600))
pg.display.set_caption("testing trained model")

running = True
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    window.fill((0, 0, 0))
    action = loaded_model(tf.expand_dims(state, 0)).numpy().squeeze(axis=0)
    state, reward, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        state, _ = env.reset()

    space_min = env.observation_space.low[0]
    space_max = env.observation_space.high[0]
    space_size = space_max-space_min

    platform_size = 10

    wind_size = window.get_size()

    platform_pos = env.platform_pos_
    platform_pos = (platform_pos-space_min)/space_size
    platform_pos = np.array([platform_pos[0], 1-platform_pos[1]])
    platform_pos = platform_pos*wind_size

    pg.draw.rect(window, (0, 255, 0), [platform_pos[0]-platform_size/2, platform_pos[1] -
                 platform_size/2, platform_size, platform_size])

    pend_pos = env.platform_pos_+env.pendulum_sim_.GetPendulumPosition()
    pend_pos = (pend_pos-space_min)/space_size
    pend_pos = np.array([pend_pos[0], 1-pend_pos[1]])
    pend_pos = pend_pos*wind_size

    pg.draw.line(window, (0, 255, 0), [platform_pos[0], platform_pos[1]], [
                 pend_pos[0], pend_pos[1]], width=5)

    text_surface = font.render("%.2f" % reward, False, (255, 255, 255))
    window.blit(text_surface, (0, 0))

    pg.display.update()

    time.sleep(1/30)

# Quit Pygame
pg.quit()
