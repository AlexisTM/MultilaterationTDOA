import setuptools
from distutils.core import setup

setup(
    name='MultilaterationTDOA',
    version='0.2.0',
    author='AlexisTM',
    author_email='alexis.paques@gmail.com',
    packages=['multilateration_tdoa'],
    scripts=[],
    url='https://github.com/AlexisTM/MultilaterationTDOA',
    license='LICENSE.txt',
    description='Multilateration library for TDoA localization.',
    long_description=open('README.md').read(),
)
