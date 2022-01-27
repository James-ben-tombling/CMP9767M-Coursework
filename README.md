# CMP9767M-Coursework
Robotics programming 21/22
## Summary 
This project use a topological naviagtion to navigate round a vineyard using the `controller.py` file, while doing so the `Thorvald` robot is stopping at certain waypoints and capturing data via the `identifier.py` using HSV thresholding and canny algorithms along with other image processing techniques, depth imagery and laser scans parallel to the RGB camera to plot the centroid of the identified bunchs aswell as mapping the vines them selves.
the clustered coordinates of the centroids filtered by the `passthrough.launch` and are then put through a density clustering algorithm called `DBScan` to count the number of grape bunch accurately. 

- Grape bunches are colured according to their elivation red = low , purple = high 
- Vines are plotted gray
- the Global Varibale `VineyardParticles` allows you to adjust the amount of laser scan you can process for the vines depending on your computing power 




## How to install and setup 
first make a folder in your home dirctory $ `cd ~ ` called `mongodb` by running the following a a new terminal: 

$ `mkdir mongodb`

then go to `/<your_catkin_ws>/src` using:

$ `cd ~/your_catkin_ws/src`

downlaod the zip link directly and unpack `cmp9767m_cw_25913881` into this directory 

go to the scripts folder via:

$ `cd ~/<you_catkin_ws>/src/cmp9767m_cw_25913882/scripts `

you need to allow access to the scripts by using this:

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
14. run $ `rosrun cmp9767m_cw_25913882 identifier.py` 
15. repeat steps ***3*** and ***4***
16. launch $ `roslaunch cmp9767m_cw_25913882 passthrough.launch`
17. repeat steps ***3*** and ***4**
18. launch $ `rosrun cmp9767m_cw_25913882 controller.py`

Watch the identifier terminal to see the total number of total bunch and RViz for they cloud point mapping of each bunch aswell as the point cloud of the vines.
