# CMP9767M-Coursework
Robotics programming 21/22

## How to run the code 

1. $`roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small`
2. in the rviz menu bar open `~/<your_catkin_ws>/uol_cmp9767m_tutorials/config/topo_nav.rviz`
3. in a new terminal $ `cd ~/<your_catkin_ws>` 
4. source the work space $ `source ./devel/setup.bash` 
5. then $`roslaunch uol_cmp9767m_tutorial topo_nav.launch`
6. repeat steps ***3*** and ***4***
7. run $ `rosrun uol_cmp9767m_tutorial imagprojection_cw.py` 
8. repeat steps ***3*** and ***4***
9. roslaunch uol_cmp9767m_tutorial passthrough.launch
10. repeat steps ***3*** and ***4**
11. rosrun uol_cmp9767m_tutorial set_topo_nav_goal.py 

