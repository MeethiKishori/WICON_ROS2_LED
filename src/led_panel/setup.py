from setuptools import find_packages, setup

package_name = 'led_pannel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/panels.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pclem12',
    maintainer_email='ifoundsingh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'leds = led_pannel.leds:main',
            'draw_cv = led_pannel.draw_cv:main',
            'panel_manager = led_pannel.panel_manager:main',
        ],
    },
)
