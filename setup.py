import os
from setuptools import setup

import minidemoKalmanFilter as pkg

pathname = os.path.join(
    os.path.abspath(os.path.dirname(__file__)),
    'README.rst')

desc = open(pathname).read()

setup(
    name = pkg.__name__,
    version = pkg.__version__,
    description = desc.split('\n')[1],
    long_description = desc,
    author = pkg.__author__,
    author_email = 'shirakawa.kenichi@gmail.com',
    url = 'https://github.com/sken10/minidemoKalmanFilter',
    packages = [pkg.__name__],
    include_package_data = True,
    license = pkg.__license__,
    keywords= 'Kalman UKF EKF LKF',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Libraries',
    ],
)
