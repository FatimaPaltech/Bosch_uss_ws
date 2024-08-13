from setuptools import find_packages, setup

package_name = 'bosch_uss_status'

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
    maintainer='paltech',
    maintainer_email='paltech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'de_mode = bosch_uss_status.de_mode:main',
            'ce_mode = bosch_uss_status.ce_mode:main',
        ],
    },
)
