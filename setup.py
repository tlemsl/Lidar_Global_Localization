from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['localizer'],
    # scripts=['bin/multi_topic_synchronizer_with_visualizable_odom.py'],
    package_dir={'': 'src'}
)

setup(**d)