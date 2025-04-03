# Comandi utili

cd /home/none/Usi_Robotics/RoboticsAss1/

ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

rm -rf build install log
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


# Spiegazione struttura progetto:

## New Custom Package

Questo pacchetto ROS 2 implementa un controllo personalizzato per il simulatore **Turtlesim**. Il sistema gestisce il comportamento di una tartaruga in due modalità operative:
- **Writing**: la tartaruga disegna una lettera seguendo le istruzioni definite nel modulo **Drawer**.
- **Chasing**: se il bersaglio (tipicamente `turtle1`) si avvicina troppo, il sistema interrompe il disegno e inizia a inseguirlo utilizzando il modulo **Turtle Behaviour** in combinazione con **Move2GoalNode**.

## Descrizione dei Componenti

### Turtle (Modello)
Il modulo `Turtle.py` definisce il modello di una tartaruga. Questo include le proprietà base, come il nome della tartaruga, e eventuali metodi utili per gestire lo stato della tartaruga nel simulatore. Utilizza funzioni helper per integrare il modello con la logica di controllo dei nodi.

### Drawer (Disegnatore)
I moduli `Drawer.py` e `USI_drawer.py` contengono la logica per disegnare le lettere.  
- **Drawer**: definisce le basi per l'esecuzione di sequenze di movimenti per riprodurre un tracciato, come ad esempio i contorni di una lettera.
- **USI_drawer**: è un'implementazione specifica (ad esempio per la lettera "U") che sfrutta la logica del Drawer per controllare i movimenti della tartaruga e ottenere il disegno desiderato.

### Turtle Behaviour (Comportamento della Tartaruga)
Il modulo `TurtleBehaviour.py` gestisce il comportamento dinamico della tartaruga. In particolare:
- **Modalità Writing**: Il nodo si sottoscrive ai topic della propria posizione e a quello del bersaglio (ad esempio `turtle1`), ed esegue un loop in cui controlla continuamente la distanza tra le due tartarughe. Finché la distanza supera una certa soglia, il nodo esegue il disegno della lettera.
- **Modalità Chasing**: Se la distanza scende al di sotto di una soglia predefinita, il nodo interrompe il disegno e passa alla modalità "chasing", chiamando il modulo `Move2GoalNode` per inseguire il target.

