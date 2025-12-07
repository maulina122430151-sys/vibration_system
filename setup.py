from setuptools import find_packages, setup

package_name = 'vibration_system'

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
    maintainer='maulina122430151-sys',
    maintainer_email='maulina.122430151@student.itera.ac.id',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'publisher_vibration = vibration_system.publisher_vibration:main',
        'subscriber_buzzer = vibration_system.subscriber_buzzer:main',
    ],
},

)
