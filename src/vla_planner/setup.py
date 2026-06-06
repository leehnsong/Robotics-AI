from setuptools import find_packages, setup

package_name = 'vla_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hancom',
    maintainer_email='hancom@todo.todo',
    description='VLA Planner Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_planner_node = vla_planner.llm_planner_node:main',
        ],
    },
)
