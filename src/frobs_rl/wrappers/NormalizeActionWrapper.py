
import numpy as np
import gym

class NormalizeActionWrapper(gym.Wrapper):
    """
    Wrapper to normalize the action space.

    :param env: (gym.Env) Gym environment that will be wrapped
    """
    def __init__(self, env):
        # Retrieve the action space
        action_space = env.action_space
        assert isinstance(action_space, gym.spaces.Box), "This wrapper only works with continuous action space (spaces.Box)"
        # Retrieve the max/min values
        self.low, self.high = action_space.low, action_space.high

        # We modify the action space, so all actions will lie in [-1, 1]
        env.action_space = gym.spaces.Box(low=-1, high=1, shape=action_space.shape, dtype=np.float32)

        # Call the parent constructor, so we can access self.env later
        super(NormalizeActionWrapper, self).__init__(env)
    
    def rescale_action(self, scaled_action):
        """
        Rescale the action from [-1, 1] to [low, high]

        :param scaled_action: The action to rescale.
        :type scaled_action: np.ndarray

        :return: The rescaled action.
        :rtype: np.ndarray
        """
        return self.low + (0.5 * (scaled_action + 1.0) * (self.high -  self.low))

    def reset(self):
        """
        Reset the environment 
        """
        # Reset the counter
        return self.env.reset()

    def step(self, action):
        """
        :param action: Action taken by the agent
        :type action: floar or int

        :return: observation, reward, is the episode over, additional informations
        :rtype: (np.ndarray, float, bool, dict)
        """
        # Rescale action from [-1, 1] to original [low, high] interval
        rescaled_action = self.rescale_action(action)
        obs, reward, done, info = self.env.step(rescaled_action)
        return obs, reward, done, info