from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name="frobs_rl",

    packages=['frobs_rl'],
    package_dir={'': 'src'},
    
    description="Flexible Robotics Reinforcement learning library.",
    url="https://github.com/jmfajardod/frobs_rl",
    keywords="robotics reinforcement-learning-algorithms reinforcement-learning machine-learning "
    "gym openai stable baselines toolbox python",
    license="MIT",
)

setup(**setup_args)