from setuptools import find_packages, setup

package_name = 'stm32_end_effector_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cjh',
    maintainer_email='jchenjb@connect.ust.hk',
    description='ROS 2 bridge for STM32 unified end-effector control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge_node = stm32_end_effector_pkg.stm32_bridge_node:main',
        ],
    },
)
