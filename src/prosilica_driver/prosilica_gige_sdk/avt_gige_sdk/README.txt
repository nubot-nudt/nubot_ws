###
### AVT PvAPI GigE SDK 1.28 for Linux
### README
###
### 01/12/2015
###

*************************************************************************
* Compilation
*************************************************************************

* This distribution supports x86, ARM, and x64 (AMD64) Linux (2.6.X). Modify the "ARCH" file located in the Examples directory before building the samples to select the proper architecture: x86, arm or x64.

* ARM binaries built compatible to arm-linux-gnueabi on Ubuntu (4.6 / 4.7) and support for hard-(HF) and soft-float(SF)

* Contact support@alliedvisiontec.com, if you can not work with this compilate on your ARM - platform

* The shared library in ./bin-pc is compiled with GCC 4.4

* Static libraries are provided for GCC 4.4, 4.5 and 4.6 and 4.7 they can be found in ./lib-pc

* Each sample code can be built as follows:

	> make sample ; make install
	
  To compile against a static library rather than the dynamic library, build as follows:
  
	> make sample-static ; make install

  The executables will be copied into the ./bin-pc folder.
  
  
*************************************************************************
* SampleViewer Running	
*************************************************************************

* SampleViewer needs the Package libtiff4-dev  
* SampleViewer needs the Package libjpeg62


*************************************************************************
* SampleViewer Compilation
*************************************************************************

* SampleViewer requires the wxGTK library. Compiled versions of SampleViewer are provided for each architecture, statically compiled against wxGTK. 

* To compile the SampleViewer source code:

  - Install the 'libgtk2.0-dev' package
  - Download wxGTK (>= 2.6) library. http://www.wxwidgets.org/downloads 
    - Compile wxGTK as Unicode. E.g:
	From within wxGTK download directory
	> mkdir static
	> cd static
	> ../configure --enable-shared=no --enable-static=yes --enable-unicode=yes
        > make
  - Edit the SampleViewer makefile to point to WxGTK build location. E.g:
        WX_DIR   = /home/you/Desktop/wxGTK-2.8.12/static	
  - > make sample ; make install
  

*************************************************************************
* CLIpConfig Running
*************************************************************************  

* CLIpConfig needs super user privileges to discover cameras in a different subnets. This can be achived by running the application as "sudo" or your distribution's super user equivalent.

  E.g. type "sudo ./CLIpConfig -l" to list all connected cameras in all subnets.

* By default AVT GigE cameras are configured to obtain their IP address automatically through DHCP. If no DHCP server is present they fall back to LLA (Link Local Address). The latter is most likely

  the case when a camera is connected to the host computer directly. The camera then uses an IP address out of the 169.254.X.X class B subnet (255.255.0.0).

  Usually any Linux distribution can handle LLA. If not, please configure the host to use a fixed IP address of the same subnet (e.g. 169.254.1.1).

  NOTE: 

    Please be careful when configuring a new fixed IP address of the camera. Recent Linux kernels filter broadcasts from other subnets to reduce broadcast traffic. This makes it impossible for CLIpConfig
	to detect the camera. If your distribution has its kernel configured in such a way, please be sure to bring both the host and the camera in the same subnet.


*************************************************************************
* Java
*************************************************************************

* The Java folder (in examples) contains a JNI interface to PvAPI and a set of samples. In the bin-pc folder, you will find a build version of the interface (libPvJNI.so). To import the samples in Eclipse:

    1. Start Eclipse & set workspace to "AVT GigE SDK/examples/Java"
    2. In the "Package Explorer", right click then select "Import". From there expand "General" and select "Existing Projects into Workspace"
       then "Next".
    3. In "Select Root directory" browse to "AVT GigE SDK/examples/Java/PvJPI" then click on "Finish". The project will appears in the "Explorer".
    4. Do the same thing for any of sample code you want, e.g. "JListCameras"
    5. Select the "JListCameras" project in the "Explorer", right click and select "Run as" -> "Java Application".
    6. In the "Select Java Application" window, scroll down to "JListCameras - Prosilica" and select it. Press "Ok". The sample code will launch,
       but fail with a java.lang.unsatisfiedLinkError exception. Terminate the app.
    7. In the "Eclipse" menu "Run", select "Run Configuration ...". Select "JListCameras" and click on the "Arguments" tab and add the following
       string to "VM arguments":
       
       -Djava.library.path="/Full/Path/To/AVT GigE SDK/bin-pc/x64"
       
       Replace Full/Path/To by the actual path to the SDK on your system. If you are on a 32bits system, replace x64 by x86.
    8. In the same window, change the "Working Directory" to "Other" and enter the exact same path as above (no need to have it quoted).
    9. Click on "Apply", then "Run".
    

*************************************************************************
* Network Optimization
*************************************************************************

* Set MTU of the GigE adapter to 9000. Set camera PacketSize = 8228. Whether your adapter can support MTU 9000 will be dependent on the NIC hardware and driver. If the MTU must be set to a lower value, adjust camera PacketSize to match.

    - (Ubuntu) Set MTU in System->Preferences->Network Connections. If 9000 is out of range for the NIC, the setting will automatically decrease to the max allowable.

    - If there is no above option, in terminal: sudo ifconfig eth0 mtu 9000
      where eth0 is the adapter used for camera. This must be done on every system startup. It may be possible to edit the /etc/network/interfaces for permanence. This will vary between distributions. Look online for more info.

* Users experiencing dropped frames or packets should run their application with root privileges. This allows the OS to boost the priority of the Prosilica driver thread.
Running as root also allows the driver to bind directly to the NIC adapter. Doing so shortens the path the packets take within the OS network stack. 
Users who feel that running their application as root compromises their system security may find the following implementation satisfactory:
	-set the executable owner as root
	-set the "setuid" permission bit on the executable
	-in code, when application starts use capset() to release all but these privileges:
		CAP_SYS_NICE
		CAP_NET_ADMIN
		CAP_NET_BROADCAST
		CAP_NET_RAW  
The application will start with all root privileges, but it will drop them immediately after startup.

* In order to use multicasting, you may have to add manually a route. The syntax is as follows:

	> sudo route -n add -net 224.0.0.0 netmask 240.0.0.0 dev eth0

where eth0 is the adapter used for camera. Multicasting will only work if the application is run as root. 
