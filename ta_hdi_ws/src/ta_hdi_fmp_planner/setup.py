from setuptools import setup

package_name = 'ta_hdi_fmp_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner_min.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS2 service planner node for TA-HDI-FMP.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_node = ta_hdi_fmp_planner.planner_node:main',
        ],
    },
)
