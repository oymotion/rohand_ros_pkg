from setuptools import find_packages, setup

package_name = 'rohand'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    maintainer='He Bin',
    maintainer_email='hebin7611@hotmail.com',
    description='ROS Package for ROHand',
    license='MIT',
    tests_require=['pytest'],
)
