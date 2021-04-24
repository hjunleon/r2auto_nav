# EG2310 Autonomous Navigation Turret
ROS2 auto_nav code for EG2310


Hello readers, on top of the code present here below are some helpful links to explain our research and references


EG2310 Algo Research: [Research Doumentation](https://docs.google.com/document/d/1sQ1H7nJIePaXPmjvzxNqh4ws5_xse7VH4cq7usQmAJE/edit?usp=sharing)

For the resultant code of all this research: [Resulatant Code](https://github.com/hjunleon/r2auto_nav/tree/main/Scripts/Workstation/scripts/scripts)


Video demonstrations are in the video demo folder


## Documentation
The Technical Documentation can be found here: [Project Documentation](https://github.com/hjunleon/r2auto_nav/blob/main/A.N.T.%20Technical%20Documentation.pdf)


## Structure and description
| File | Functionality |
| --- | --- |
| `RPi/scripts/scripts/thermal.py` | Initialisation code for Thermal Camera |
| `RPi/scripts/scripts/firing.py` | Initialisation code for Actuation |
| `Workstation/scripts/scripts/command_pub.py` | Code for thermal interpolation and controlling all the nodes |
| `Workstation/scripts/scripts/plotting_sub.py` | Code for plotting thermal data |
| `Workstation/scripts/scripts/testing.py` | Main Planning code |
| `Workstation/scripts/scripts/dwa_planner.py` | Implementation of DWA Planning Algorithm |


## Team
Members:
1. Koh Yang Kai
2. Jasshan Kumeresh
3. Gowthaman Aravindan
4. Hoe Jun leong
5. Ge Yuwen

Class of EG2301 Group 3 (20/21 Sem 2)
