# COmendi utili

cd /home/none/Usi_Robotics/RoboticsAss1/

ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

colcon build --symlink-install

source install/setup.bash


ros2 pkg list | grep <pakagename>

### Creazione di un nuovo pakage: 
ros2 pkg create --build-type ament_python <new_pakage_name> 

Mettiamo che abbiamo creato un nuovo file ass1 nel pakage src/new_pakage_name dobbiamo modifucare il file src/new_pakage_name/setup.py 

come segue:
```python
    from setuptools import setup

    package_name = 'new_pakage_name'

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
        maintainer='il_tuo_nome',
        maintainer_email='il_tuo_email',
        description='Descrizione del tuo pacchetto',
        license='Licenza del pacchetto',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'ass1 = new_pakage_name.Ass1:main'
            ],
        },
    )
```

Bulizia file build: 
rm -rf build/ install/ log/