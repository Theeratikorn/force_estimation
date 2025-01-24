from setuptools import setup

package_name = 'dynamic_joint_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Package for logging dynamic joint states to CSV.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_joint_state_logger = dynamic_joint_logger.dynamic_joint_state_logger:main',
        ],
    },
)

