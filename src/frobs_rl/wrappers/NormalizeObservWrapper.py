
import numpy as np
import gym

class NormalizeObservWrapper(gym.Wrapper):
    """
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
        Scale the observation from [low, high] to [-1, 1]
        (no need for symmetric observation space)
        :param observation: (np.ndarray)
        :return: (np.ndarray)
        """
        return ((observation - self.low) * (1.0/(0.5*(self.high-self.low)))) - 1.0

    def reset(self):
        """
        Reset the environment 
        """
        # 
        observation = self.env.reset()
        scaled_obs = self.scale_observation(observation)
        return scaled_obs

    def step(self, action):
        """
        :param action: ([float] or int) Action taken by the agent
        :return: (np.ndarray, float, bool, dict) observation, reward, is the episode over?, additional informations
        """
        observation, reward, done, info = self.env.step(action)
        # Rescale observation from [low, high] to [-1, 1] interval
        scaled_obs = self.scale_observation(observation)
        return scaled_obs, reward, done, info