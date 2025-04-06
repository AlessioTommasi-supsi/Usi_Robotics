from setuptools import setup, find_packages

package_name = 'new_custom_pakage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='none',
    maintainer_email='none@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ass1 = new_custom_pakage.Ass1:main',
            'spown = new_custom_pakage.controller.TurtleSpawner:main'
        ],
    },
)
