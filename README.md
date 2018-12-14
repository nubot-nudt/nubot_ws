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
   
   `$ catkin_make`

## RUN
### For real robot applications
1. All components:   
   ` $ roslaunch nubot_common nubot.launch`
2. Joy stick and hardware controller   
   ` $ roslaunch nubot_hwcontroller nubot_hwcontroller.launch`   
   ` $ rosrun nubot_hwcontroller nubot_teleop_joy`
3. Show images from the cameras   
   ` $ rqt_image_view`
   
### For simulation
1. Please edit the file: [core.hpp][1] and uncomment '#define SIMULATION'; then compile the code again.
2. Run the following commands:   
` $ ./src/cyan_spawn_model.sh` for cyan robots,   
or   
` $ ./src/magenta_spawn_model.sh` for magenta robots.

## Error & Fix
1. Problem: nubot_hwcontroller shows an error: 'pid *** died'.   
Solution: ` $ sc devel/lib/nubot_hwcontroller/nubot_hwcontroller_node   `
The command 'sc' is an alias of 'sudo setcap cap_net_raw=ep'

2. Problem: CMake Error at nubot/color_segment/CMakeLists.txt:40 (Qt5_WRAP_CPP): Unknown CMake command "Qt5_WRAP_CPP".    
Explanation: using color_segment package to calibrate color-related parameters. This requires Qt since it has a Qt GUI.   
Solution 1: You could just delete this package since it does not interfere with other parts. So the compile will go on.   
Solution 2: Go to src/nubot/color_segment and edit CMakeLists.txt line 4; change the path to your Qt5Widgets.   

3. Problem: hwcontroller_node.cpp:6:21: fatal error: ncurses.h: No such file or directory     
Solution: `$ sudo apt-get install libncurses5-dev`   

4. help files are located in /doc folder; please refer to them   


## Contact
NuBot Team: nubot.nudt@outlook.com or nubot.nudt@gmail.com

[1]: src/nubot/nubot_common/core/include/nubot/core/core.hpp
