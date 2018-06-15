from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	scripts = ['src/uav_restriction_node.py'],
)

setup(**d)
