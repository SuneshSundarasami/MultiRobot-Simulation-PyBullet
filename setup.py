from setuptools import find_packages, setup

package_name = 'robile_pybullet'

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
    maintainer='sunesh',
    maintainer_email='sunesh@outlook.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'test_movement = {package_name}.test_movement:main',
            f'robile_sim = {package_name}.robile_sim:main',
        ],
    },
    
)
