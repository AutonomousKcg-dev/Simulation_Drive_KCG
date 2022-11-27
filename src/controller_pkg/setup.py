from setuptools import setup

package_name = 'controller_pkg'
submodules = "controller_pkg/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eyal',
    maintainer_email='eyalh2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "switch = controller_pkg.switch_vaya:main",
        "controller = controller_pkg.controller:main",
        ],
    },
)
