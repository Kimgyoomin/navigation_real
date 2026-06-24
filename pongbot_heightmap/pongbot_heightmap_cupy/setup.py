from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["pongbot_heightmap_cupy", "heightmap_package_cupy.plane_segment", "pongbot_heightmap_cupy.plugins",], package_dir={"": "script"},
)

setup(**setup_args)
