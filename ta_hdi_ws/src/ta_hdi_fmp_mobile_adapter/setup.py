from setuptools import setup

package_name = 'ta_hdi_fmp_mobile_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mobile_adapter.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Pure pursuit adapter for mobile robot tracking TA-HDI-FMP paths.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mobile_adapter_node = ta_hdi_fmp_mobile_adapter.mobile_adapter_node:main',
            'benchmark_runner = ta_hdi_fmp_mobile_adapter.benchmark_runner:main',
        ],
    },
)
