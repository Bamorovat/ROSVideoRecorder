## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['VideoRecorder'],  # Replace with your package name
    package_dir={'': 'src'},      # Directory where your package's Python module is located
    scripts=['scripts/VideoRecorder.py']  # Replace with the path to your executable script
)

setup(**setup_args)
