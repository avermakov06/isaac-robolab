"""Script to train RL agent with Stable Baselines3."""

"""Launch Isaac Sim Simulator first."""

import argparse
from pathlib import Path


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with Stable-Baselines3.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--log_interval", type=int, default=100_000, help="Log data every n timesteps.")
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
parser.add_argument(
    "--keep_all_info",
    action="store_true",
    default=False,
    help="Use a slower SB3 wrapper but keep all the extra training info.",
)

import gymnasium as gym
from gymnasium.wrappers import TimeLimit
import numpy as np
import os
import random

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, LogEveryNTimesteps
from stable_baselines3.common.vec_env import VecNormalize


LOG_DIR = "./sb3_logs"

import wandb
from wandb.integration.sb3 import WandbCallback

wandb.init(
    project="isaac-ppo",
    sync_tensorboard=True,
)


from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True,
})


def main():
    """Train with stable-baselines agent."""
    # create isaac environment
    # env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    
    from env import AgibotGymEnv
    env = AgibotGymEnv()
    env = TimeLimit(env, 2048)
    
    # create agent from stable baselines
    agent = PPO("MultiInputPolicy", env, verbose=1, tensorboard_log=LOG_DIR, n_steps=2048)

    callbacks = [WandbCallback(), LogEveryNTimesteps(n_steps=1000)]

    # train the agent
    agent.learn(
        total_timesteps=2000000,
        callback=callbacks,
        progress_bar=True,
        log_interval=1,
    )
    
    env.close()


if __name__ == "__main__":
    main()
