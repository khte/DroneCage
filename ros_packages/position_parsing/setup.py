from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	scripts = ['src/position_parser_node.py', 'src/position_parser_node_with_crc.py'],
)

setup(**d)
