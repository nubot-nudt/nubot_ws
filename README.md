# MSL soccer robot code of NuBot team of National University of Defense Technology
## Package Summary   

- Maintainer status: maintained
- Maintainer: [NuBot Team](https://www.trustie.net/organizations/23?org_subfield_id=108)
- Author: [NuBot Team](https://www.trustie.net/organizations/23?org_subfield_id=108)
- License: Apache
- Bug / feature tracker: https://github.com/nubot-nudt/nubot_ws/issues
- Source: git https://github.com/nubot-nudt/nubot_ws (branch: master)
   
## Note
	The strategy part in nubot_control package has been removed. 
	But you can still compile the package successfully.

## Compile
   `$ sudo chmod +x configure`
   
   `$ ./configure`
   
   `$ catkin_make -j1`

## RUN
1. All components:
   $ roslaunch nubot_common nubot.launch
2. Joy stick and hardware controller
   $ roslaunch nubot_hwcontroller nubot_hwcontroller.launch
   $ rosrun nubot_hwcontroller nubot_teleop_joy
3. Show images from the cameras
   $ rqt_image_view

## Error & Fix
1. nubot_hwcontroller shows an error: 'pid *** died'.
Fix: $ sc devel/lib/nubot_hwcontroller/nubot_hwcontroller_node
2. help files are located in /doc folder; please refer to them


## Note
1. If you want to use the simulation function, please edit the file: src/nubot/nubot_common/core/include/nubot/core/core.hpp, and uncomment '#define SIMULATION'. Then compile the code again.For furthur tutorial, see related documentation in other repositories(i.e. 'gazebo_visual' or 'single_nubot_gazebo')
2. contact info: nubot.nudt@outlook.com
