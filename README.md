# CMP9767M-Coursework
Robotics programming 21/22
## How to install and setup 
first make a folder in your home dirctory $ `cd ~ ` called `mongodb` by running the following a a new terminal: 

$ `mkdir mongodb`

then go to `/<your_catkin_ws>/src` using:

$ `cd ~/your_catkin_ws/src`

downlaod the zip link directly and unpack `cmp9767m_cw_25913881` into this directory 

go to the scripts folder via:

$ `cd ~/<you_catkin_ws>/src/cmp9767m_cw_25913882/scripts `

need to allow access to the scripts by using this:

$ ` chmod +x identifier.py controller.py`

then change dirctory back to `~/<your_catkin_ws>` and build your workspace 

$ `cd ~/<your_catkin_ws> `

$ `catkin_make`





## How to run the code 

1. in a new terminal $ `source /opt/ros/melodic/setup.bash` 
2. then $`roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small`
3. in a new terminal $ `cd ~/<your_catkin_ws>` 
6. source the workspace $ `source ./devel/setup.bash` 
7. then $`roslaunch cmp9767m_cw_25913882 topo_nav.launch`
8. an error saying `Desired pointset 'test' not in datacentre, try :1` will appear dont worry 
9. repeat steps ***3*** and ***4***
10. $ `rosrun topological_utils load_yaml_map.py $(rospack find uol_cmp9767m_tutorial)/maps/test_mod.yaml`
11. in the rviz menu bar open config `~/<your_catkin_ws>/src/uol_cmp9767m_tutorials/config/topo_nav.rviz` 
12. (close the previous without saving when promted)
13. repeat steps ***3*** and ***4***
14. run $ `rosrun cmp9767m_cw_25913882 imagprojection_cw.py` 
15. repeat steps ***3*** and ***4***
16. launch $ `roslaunch cmp9767m_cw_25913882 passthrough.launch`
17. repeat steps ***3*** and ***4**
18. launch $ `rosrun cmp9767m_cw_25913882 set_topo_nav_goal.py`

