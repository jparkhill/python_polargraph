# sudo pip install -e .
#

# from __future__ import absolute_import, print_function
from distutils.core import setup, Extension
import os

setup(
    name='python_polargraph',
    python_requires='>3.5.2',
    version='0.1',
    description='A pythonic art-maker.',
    url='',
    author='john parkhill',
    author_email='john.parkhill@gmail.com',
    license='CC-Share alike',
    packages=[],
    package_dir={},
    # zip_safe=False,
    # include_package_data=True,
    ext_modules=[],
    install_requires=[
        'numpy', 'scipy', 'matplotlib', 'svgwrite', 'imageio',
        'adafruit-blinka', 'adafruit-circuitpython-busdevice', 'adafruit-circuitpython-register'
    ])
