import gymnasium as gym
from typing import Any
class TimeLimitWrapper(gym.Wrapper):
  """
  Wrapper to limit the number of steps per episode.

  :param env: (gym.Env) Gym environment that will be wrapped
  :param max_steps: (int) Max number of steps per episode
  """
  def __init__(self, env, max_steps=100):
    # Call the parent constructor, so we can access self.env later
    super(TimeLimitWrapper, self).__init__(env)
    self.max_steps = max_steps
    # Counter of steps per episode
    self.current_step = 0
  
  def reset(self, seed: Any =None, options: Any =None, **kwargs):
    """
    Reset the environment 
    """
    # Reset the counter
    self.current_step = 0
    return self.env.reset(seed=seed, options=options, **kwargs)

  def step(self, action):
    """
    :param action: Action taken by the agent
    :type action: [float] or int

    :return: observation, reward, is the episode over, additional informations
    :rtype: (np.ndarray, float, bool, dict)
    """
    self.current_step += 1
    observation, reward, terminated, truncated, info = self.env.step(action)
    
    # Overwrite the done signal when 
    if self.current_step >= self.max_steps:
      terminated = True
      truncated = True
      # Update the info dict to signal that the limit was exceeded
      info['time_limit_reached'] = True
      self.info['is_success'] = 0.0

    return observation, reward, terminated, truncated, info