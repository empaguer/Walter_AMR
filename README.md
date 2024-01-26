# AMR - Walter
## Guia para uso de Walter

[![N|Solid](https://upload.wikimedia.org/wikipedia/commons/9/92/Espol_Logo_2023.png)](http://www.cidis.espol.edu.ec)

Este proyecto se trata sobre la construcción y programación de un robot autónomo con funcionalidades autónomas de manera local y con controlador externo (App movil), este robot (bautizado como Walter) es capaz de:
- Mapear: Como primer escenario se establece el mapeo de un escenario o local
- Navegar: Una vez establecidos las coordenadas deseadas a partir del rviz, puede navegar y esquivar obstáculos globales y dinámicos

La arquitectura del robot está basada en utilizar una computadora Maestro y otra esclavo para correr el funcionamiento de manera local o utilizando **Rosbridge** para comunicar el controlador (computadora) con la App movil, para esto es necesario que ambas partes estén conectadas a la misma red Wifi.

Las bases de funcionamiento de este proyecto se basan en [AMR CoviBot][Steeven], el cual usó la misma plataforma robótica para realizar tareas de desinfección con lámparas UV.



## Prerequisitos

La funcionalidad de Walter trabaja con la versión de Ros Noetic, esta versión funciona con la versión de Ubuntu 20.04 , si se desea usar el código Legacy (Jetson Nano), se puede seguir el tutorial presente en la documentación de [AMR CoviBot][Steeven].

Para poder utilizar y modificar parámetros del Robot, se debe tener buenas bases acerca de instalación de paquetería, Linux y sobretodo ROS, se recomienda fuertemente leer el curso introductorio de ROS impartido por FunPythonEc [Tutorial FuncPython][Funpython] , y también se recomienda fuertemente adquirir el curso: [Ros Essentials][Udemy] de Udemy.

### Set up Sistema Operativo
1. Descargar Ubuntu e instalarlo.

Descargar la versión Ubuntu 20.04 con el siguiente link [Image Ubuntu][Image].

Ya una vez descargado, seguir guia de instalación [Guia Ubuntu][Guide].

2. Instalar Ros Noetic.

Directamente seguir el tutorial: [Guia Instalación Ros Noetic][Noetic]

### Paquetes
Cuando se clona inicialmente el proyecto, es muy probable que no exista cierta paquetería inicial en el Sistema Operativo, una vez detectada la paquetería faltante se puede instalarlo con el siguiente comando en un cmd
```sh
sudo apt install ros-$ROS_DISTRO-package-name
```

Para inicializar el espacio de trabajo (workspace de ROS), es necesario ubicarse en alguna carpeta (preferiblemente home) y correr el siguiente comando, en este caso el workspace se llamará “catkin_ws” pero puede tener cualquier nombre:
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
```
La configuración de Walter se basa en el modelo de robot turtlebot3, para tener configurado este ambiente es necesario correr lo siguiente:
```sh
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
en el archivo .bashrc se debería ver lo siguiente:
```sh
export = TURTLEBOT3_MODEL =burguer
source /opt/ros/noetic/setup.bash
source ~/amr/amr_ws/devel/setup.bash
```

Un problema muy común al instalar la paquetería del proyecto son conflictos con el paquete Cartographer, si se tiene inconvenientes con este paquete, revisar directamente su guía de instalación con el siguiente link: [Carto Instalation][Carto]
Para evitar cualquier problema con las cámaras o productos Intel, asegurarse de seguir la guía oficial encontrada en: [Realsense][Realsense]

## Funcionamiento Simulado
El funcionamiento Simulado de Walter se Basa en 3 paquetes modificables y un paquete de **robotPV_PV1_description** donde se tiene los meshes y configuraciones de Joints del Robot.
Los paquetes modificables son:
- arlobot_sim_gazebo: Contiene la información de los Mundos del Robot y plugins de sensores
Markdown is a lightweight markup language based on the formatting conventions
that people naturally use in email.
- arlobot_sim_slam : Contiene diferentes métodos SLAM principalmente para Mappear
- arlobot_sim_navigation : Contiene configuraciones para la navegación de Walter y su funcionalidad completa de Robot Mesero

Para iniciar una simulación se debe correr el siguiente comando:
```sh
roslaunch arlobot_sim_gazebo arlobot_restaurant.launch world:=ecuadorian_final
```
En este caso “ecuadorian_final” corresponde al archivo ecuadorian_final.world que se debe encontrar en la carpeta /worlds, si se desea poner otro ambiente de simulación, simplemente añadir el archivo .world deseado en el directorio correcto (Si se desea correr la operación de Walter, es necesario introducir los marcadores fiduciales de Apriltag ya en el .world).
Para cargar los marcadores fiduciales se debe hacer lo siguiente:
1.	Abrir un archivo .world con el roslaunch arlobot_sim_gazebo arlobot_restaurant.launch world:=myworld.
2.	Eliminar el Robot de la simulación de gazebo, dejando solamente el mapa.
3.	Añadir los marcadores fiduciales en las mesas o estaciones deseadas.
4.	Guardar el nuevo archivo .world (Este ya contendrá los marcadores añadidos)

Para cargar los marcadores en la simulación se recomienda seguir el siguiente video: [Tutorial Fiducial][Tutorial]
Para iniciar el mapeo se debe correr el siguiente comando (gazebo ya debe estar abierto).
```sh
roslaunch arlobot_sim_slam arlo_simulation_slam.launch slam_methods:=cartographer
```
Dentro de los slam_methods se puede utilizar cartographer, gmapping, hector (hector mapping)
Para mover el robot se usa directamente el comando teleop, en otra terminal escribir:
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Una vez terminada la tarea de Mapeo, para finalizar el mapa simplemente se debe escribir el siguiente código en otra terminal:
```sh
rosrun arlobot_sim_slam cartographer_finisher.py “nombre_mapa”
```
Este commando automáticamente crea los archivos _.yaml .pgm y .bag.pbstream_ en el directorio /arlobot_sim_navigation/maps/
Para realizar las tareas de navegación es necesario cerrar por completo los nodos de mapeo y correr uno de los metodos disponibles, los cuales son:
-	Allin.launch: Este archivo combina amcl con cartographer_navigation, mejor opción para simulación.
-	Arlobot_carto_navigation: Utiliza solamente cartographer_navigation.
-	Arlobot_navigation: Utiliza solamente amcl

El código de la consola debería ser:
```sh
roslaunch arlobot_sim_navigation metodo_disponible map:=”nombre_mapa”
```
Donde “nombre_mapa”es el nombre del mapa anteriormente grabado con el nodo cartographer_finisher, si se intenta poner un nombre_mapa no existentente, el nodo de navegación no funcionará.

Con este nodo activo, se puede realizar tareas de navegación y localización, para correr el nodo de Walter Mesero se debe primeramente configurar la posición de las estaciones.
Con el mapa de rviz visualizándose, utilizar la herramienta publish point para ver su ubicación relativa en el mapa, de ahí se obtiene la ubicación deseada del punto para realizar un movimiento.

Una vez identificados los puntos, se debe crear un archivo “nombre_mapa.csv” en la ubicación /arlobot_sim_navigation/coordinates/ donde tendrá todas las ubicaciones de puntos de interés, en este archivo se deben guardar las localizaciones, seguir nomenclatura del example.csv

Cuando ya se tiene el archivo .csv creado, se puede correr directamente el nodo de funcionamiento de Walter, el cual realizará:

1.	Movimiento hacia la estación deseada.
2.	Calibración de orientación con Apriltag
3.	Movimiento hacia la estación Base
4.	Calibración de estación base con Apriltag

Para correr esta funcionalidad se debe correr en el cmd el siguiente comando:
```sh
rosrun arlobot_sim_navigation anothertest.py “nombre_mapa” “estación”.
```
Donde “estación” corresponde al ID de la estación donde se quiere hacer el movimiento de Walter.
Si se desea ver un funcionamiento Inicial, se puede correr el ejemplo base:
```sh
roslaunch arlobot_sim_gazebo arlobot_restaurant.launch world:=cafee_tags
roslaunch arlobot_sim_navigation  arlobot_carto_navigation.launch map:=cafee_tags
rosrun arlobot_sim_navigation anothertest.py cafee_tags 1
```
Para asegurarse que cafee_tags sea un mapa válido, es necesario ya tener cargado los tags (marcadores apriltag) en el modelo, si los tags no logran ser cargados, se recomienda modificar el mapa original cafee.world y a ese mapa añadir los marcadores fiduciales y seguir el tutorial de navigation para establecer las coordenadas deseadas.

## Funcionamiento Real

Para el funcionamiento de Walter, su funcionalidad es igual a su versión implementada, primeramente se debe ejecutar el siguiente comando:
```sh
roslaunch arlobotcar_nav laser_esp_bringup.launch
```
Este launcher permitirá alzar todos los nodos iniciales, esp, lidar, cámaras.
Para realizar tarea de mapeo se debe correr el siguiente comando:
```sh
roslaunch arlobotcar_nav arlobot_map_cartographer.launcher
```

Una vez realizado este comando, se puede utilizar el Joystick para mover al robot y controlar su velocidad en x y su giro (yaw).
Una vez se ha mapeado, se puede grabar el mapa con el comando:
```sh
rosrun arlobotcar_nav cartographer_finisher.py “nombre_mapa”
```

Este comando realiza la misma funcionalidad que su versión simulada.
Se inicializa la navegación con el comando:
```sh
roslaunch arlobotcar_nav arlobot_carto_navigation.launch map:=nombre_mapa
```

Igual que en la simulación, una vez ya se ha puesto sus estaciones deseadas en el directorio /arlobotcar_nav/coordinates/nombre_mapa.csv, se puede correr su funcionalidad de Walter Mesero con el comando:
```sh
rosrun arlobotcar_nav operate_walter.py nombre_mapa ID
```

La operación a realizar será la siguiente:
1.	 Movimiento hacia la estación deseada.
2.	Calibración de orientación con Apriltag
3.	Espera de Feedback de la cámara D435 (Thumbs up for 5 seconds)
4.	Movimiento hacia estación Base
5.	Calibración de orientación con Apriltag

## Funcionalidad con App Movil
Para trabajar con la app móvil, es necesario inicialmente levantar ciertos nodos de funcionamiento, se deben correr los siguientes comandos inicialmente:
```sh
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun web_video_server web_video_server _port:=8090
rosrun arlobotcar_nav bringup_topics.py
rosrun arlobotcar_nav map_handler.py
```

Con estos tópicos encendidos, la aplicación Movil es capaz de intercambiar información por nodos para realizar operaciones de mapeo, movimiento, navegación y visualizar la cámara.
## Funcionalidades paqueterías especificas
Si se desea saber la funcionalidad de los paquetes de este proyecto, revisar [DEMOS.md][Demo]

## Contacto
Si se tiene algún problema o duda, contactar a los correos: crisva212v@gmail.com ,  empaguer@espol.edu.ec






[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
   [Steeven]: <https://gitlab.com/sasilva1998/amr>
   [test]: <https://gitlab.com/MigMaroto/amr/-/tree/master?ref_type=heads>
   [Funpython]: <https://github.com/FunPythonEC/curso_intro_ros?tab=readme-ov-file>
   [Udemy]: <https://www.udemy.com/course/ros-essentials/>
   [Image]: <https://releases.ubuntu.com/20.04/>
   [Guide]: <https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview>
   [Noetic]: <https://wiki.ros.org/noetic/Installation/Ubuntu>
   [Carto]: <https://qiita.com/devemin/items/1723058cf3bac85aaa0b>
   [Realsense]: <https://github.com/IntelRealSense/realsense-ros>
   [Tutorial]: <https://www.youtube.com/watch?v=A3fJwTL2O4g>
   [Demo]: <https://github.com/empaguer/Walter_AMR/blob/master/DEMOS.md>
