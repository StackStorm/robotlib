# coding: utf-8
from setuptools import setup, find_packages

NAME = "robotlib"
VERSION = "0.1.0"

# To install the library, run the following
#
# python setup.py install
#
# prerequisite: setuptools
# http://pypi.python.org/pypi/setuptools

REQUIRES = [
    'pyserial',
    'rpi.GPIO'
]

setup(
    name=NAME,
    version=VERSION,
    description="Robot lib",
    author_email="matthew99@gmail.com",
    url="",
    keywords=["Robot Arm"],
    install_requires=REQUIRES,
    packages=find_packages(),
    include_package_data=True,
    long_description="""Library for interacting with PhantomX Reactor
    """
)
