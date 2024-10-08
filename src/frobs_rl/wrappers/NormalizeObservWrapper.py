
import numpy as np
import gymnasium as gym
from typing import Any
class NormalizeObservWrapper(gym.Wrapper):
    """
    Wrapper to normalize the observation space.

    :param env: (gym.Env) Gym environment that will be wrapped
    """
    def __init__(self, env):
        # Retrieve the observation space
        observation_space = env.observation_space
        assert isinstance(observation_space, gym.spaces.Box), "This wrapper only works with continuous observation space (spaces.Box)"
        # Retrieve the max/min values
        self.low, self.high = observation_space.low, observation_space.high

        # We modify the action space, so all actions will lie in [-1, 1]
        env.observation_space = gym.spaces.Box(low=-1, high=1, shape=observation_space.shape, dtype=np.float32)

        # Call the parent constructor, so we can access self.env later
        super(NormalizeObservWrapper, self).__init__(env)
    
    def scale_observation(self, observation):
        """
        Scale the observation from [low, high] to [-1, 1].
        
        :param observation: Observation to scale
        :type observation: np.ndarray

        :return: scaled observation
        :rtype: np.ndarray
        """
        # TODO (jose.fajardo): Remove this when the issue with the observation is solved
        if isinstance(observation, tuple):
            observation = observation[0]

        observation = np.array(observation)
        
        # Ensure observation, self.low, and self.high have the same shape
        if observation.shape != self.low.shape or observation.shape != self.high.shape:
            raise ValueError(f"Observation shape {observation.shape} doesn't match low {self.low.shape} or high {self.high.shape}")
        
        return ((observation - self.low) * (1.0/(0.5*(self.high-self.low)))) - 1.0

    def reset(self, seed: Any =None, options: Any =None, **kwargs):
        """
        Reset the environment 
        """
        # 
        observation = self.env.reset(seed=seed, options=options, **kwargs)
        scaled_obs = self.scale_observation(observation)
        return scaled_obs, {}

    def step(self, action):
        """
        :param action: Action taken by the agent
        :type action: float or int

        :return: observation, reward, is the episode over, additional informations
        :rtype: (np.ndarray, float, bool, dict)
        """
        observation, reward, terminated, truncated, info = self.env.step(action)
        # Rescale observation from [low, high] to [-1, 1] interval
        scaled_obs = self.scale_observation(observation)
        return scaled_obs, reward, terminated, truncated, info