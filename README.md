# CMP9767M-Coursework
Robotics programming 21/22

## How to run the code 

1. in a new terminal $ `source /opt/ros/melodic/setup.bash` 
2. then $`roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small`
3. in the rviz menu bar open config `~/<your_catkin_ws>/src/uol_cmp9767m_tutorials/config/topo_nav.rviz`
4. in a new terminal $ `cd ~/<your_catkin_ws>` 
5. source the work space $ `source ./devel/setup.bash` 
6. then $`roslaunch uol_cmp9767m_tutorial topo_nav.launch`
7. repeat steps ***3*** and ***4***
8. run $ `rosrun uol_cmp9767m_tutorial imagprojection_cw.py` 
9. repeat steps ***3*** and ***4***
10. launch $ `roslaunch uol_cmp9767m_tutorial passthrough.launch`
11. repeat steps ***3*** and ***4**
12. launch $ `rosrun uol_cmp9767m_tutorial set_topo_nav_goal.py` 

