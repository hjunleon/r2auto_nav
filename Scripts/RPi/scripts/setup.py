from setuptools import setup

package_name = 'scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coldyoungguy',
    maintainer_email='coldyoungguy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_test = scripts.thermal_test:main',
            'thermal_interpolate = scripts.thermal_interpolate:main',
            'command_pub = scripts.command_pub:main',
        ],
    },
)