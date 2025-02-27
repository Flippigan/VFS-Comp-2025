What precipitated a fresh install was slow simulation speeds, AND wiping the src directory on my previous laptop, which caused me to loose all config and sdf world files. I was following a Claude instructions blindly, causing me not to read what it was saying before running the rmdir command. NEVER do that again.


1. I'm installing the entire ROS2 WS from these [instructions](https://ardupilot.org/dev/docs/ros2-gazebo.html) 
	1. cd ~/ardu_ws
	2. vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
2. [build SITL ](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)installation from the ardupilot repo installed from above (skip to instructions after cloning ardupilot since we already did that)
	1. added `source ~/Documents/ardu_ws/src/ardupilot/Tools/completion/completion.bash` to .profile; which is a file similar to .bashrc, except instead of the commands in it being run for every new terminal created, only runs the commands when your computer starts up. 
	2. Next we use WAF to build
		1. Used `./waf configure --board sitl` , `./waf copter` , `./waf plane`
		2. Need to use 
3. Install ROS2 [dependencies that ardupilot depends on](https://ardupilot.org/dev/docs/ros2.html#ros2)
4. ROS2 with SITL working after following [these instructions](https://ardupilot.org/dev/docs/ros2-sitl.html#ros2-sitl)
	1. The instructions are missing key dependencies installs; when you get an error trying to run SITL with ROS2 for the first time, simply use a GPT to analyze the output and install the missing dependencies.
	2. See the Node list with `ros2 node list`
	3. See what topics are available on each node with `ros2 node info /ap` or whatever nodes are running 
5. trying to run 'colcon build --packages-up-to ardupilot_gz_bringup' resulted in the following error
		Could not find a package configuration file provided by "gz-cmake3" with
		  any of the following names:
		    gz-cmake3Config.cmake
		    gz-cmake3-config.cmake
	
	1. Which lead me to gazebo had not been installed. Because of course, the documentation assumes you have already installed gazebo from this guide "[The purpose of this guide is to show how to integrate ArduPilot with Gazebo using ROS 2.](https://ardupilot.org/dev/docs/ros2-gazebo.html)"
	2. Therefore we must follow [these instructions](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) on installed gazebo for our correct environment to initiate the ROS2 + gazebo integration. 
	3. Then install the [ardupilot Gazebo Plugin from here](https://github.com/ArduPilot/ardupilot_gazebo/blob/main/README.md) 
		1. Since we already made ardu_ws and cloned ardupilot_gazebo into it, we ignore step 2. From then on remember that this is our environment
		2. Adjust the `export` commands in configuring the environment to the correct directory 
			1. export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/Documents/ardu_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
			2. export GZ_SIM_RESOURCE_PATH=$HOME/Documents/ardu_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
	4. Next we want to begin using gazebo with ROS2 + SITL; https://ardupilot.org/dev/docs/ros2-gazebo.html
		1. Problem; system was crashing when trying to `colcon build --packages-up-to ardupilot_gz_bringup`, fixed by using `VERBOSE=1 colcon build --executor sequential --packages-up-to ardupilot_gz_bringup 2>&1 | tee build_log.txt`
		2. is_runway.launch.py` doesn't launch Mavproxy, so I have no way of controlling the aircraft. I can see the ROS topics running correctly, but I don't know how to control the aircraft
			1. Next step after controlling aircraft is to 
				1. Figure out how to get camera data to and from the model to our code
					1. confused about where to start with implementing a simulated camera module in the Gazebo, meaning I dont know which folder to put it in, or what to put (I think it's a update to the SDF file)
					2. Then I need to learn [dronekit](https://dronekit.netlify.app/) and implement this [code for control ](https://claude.ai/chat/918a9e1a-ce21-4d99-875d-31e0cee11dfc)
						1. ROS2 NODES ARE HOW WE GET CAMERA DATA FROM GAZEBO TO PYTHON AND OPENCV. I know from [this video](https://www.youtube.com/watch?v=5eg3u7P1n4g) which contains the full python code
	![[Pasted image 20241229110913.png]]
							1. Sanket Sharma is using rospy, what other options are there?
								1. I need to figure out how to get data from model.SDF camera module to my python code		
									1. Figure out which Model and World file the gazebo sim is using. [[ROS 2 with ardupilot | this note explaining the purpose of each package in SRC directory]] could help identify which is which, after that use trial and error
										1. WORLD: `Documents/ardu_ws/src/ardupilot_gz/ardupilot_gazebo/worlds/iris_runway.sdf`
										2. MODEL: `Documents/ardu_ws/src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf`
				1. Figure out which folder / SDF files are used by this startup sequence ([source ardupilot_gz](https://github.com/ArduPilot/ardupilot_gz#ardupilot_gz)) 
					1. We used cursor to analyze the startup file the startup sequence and found the paths to the WORLD sdf file is located in `Documents/ardu_ws/src/ardupilot_gz/ardupilot_gazebo/worlds/iris_runway.sdf` 
					2. We found that the <span style="color:rgb(0, 176, 80)">MODEL sdf file</span> used by the WORLD above`iris_runway.sdf` <span style="color:rgb(0, 176, 80)">is located at </span>`Documents/ardu_ws/src/ardupilot_gazebo/models/iris_with_gimbal/model.sdf` because in the WORLD sdf file, at line 76 we have `<uri>model://iris_with_gimbal</uri>`
					3. With GREP, cursor then searched for this TEXT in ALL FILES. So it literally searched for all instances of `iris_with_gimbal` in the ardu_ws. Which it found multiple instances of, within the `iris_with_gimbal`directories there are MODEL sdf files, and defined in line 3 of BOTH MODEL sdf files there is `<model name="iris_with_gimbal">`
					4. I'm not sure how the AI identified the correct `iris_with_gimbal` MODEL SDF file given that they are both located at the following  <span style="color:rgb(0, 176, 240)">differing locations </span>
						1. /home/finn/Documents/ardu_ws<span style="color:rgb(0, 176, 240)">/src</span>/ardupilot_gazebo/models/iris_with_gimbal/model.sdf
						2. /home/finn/Documents/ardu_ws/<span style="color:rgb(0, 176, 240)">install</span>/ardupilot_gazebo/share/ardupilot_gazebo/models/iris_with_gimbal/model.sdf
					5. find out what gz_system_resource path does, if I can remove the gz_ws paths, and how having so many paths effects trying to edit the correct world and SDF files / is it a good idea to delete and simplify these paths** I think it may have something to do with the GZ_path stuff, but also look into the underlying differences between <span style="color:rgb(0, 176, 240)">install</span> and <span style="color:rgb(0, 176, 240)">src</span>
				2. get the VTOL SDF model into the correct DIR to work with ROS, 
				3. re-instate the circles to the world SDF file, 
				4. re-instate the VFS comp GPS to the SITL mav proxy map. 
				5. add a gimbal to the VTOL, 
				6. get the camera data to a python script.
					1. We have the pipeline... ROS2 -> ROSpy -> pymavlink (websites and repo's in github)
					2. [Open CV bridge](http://wiki.ros.org/cv_bridge) is one option
					3. Investigate what the IRIS_gimbal SDF model uses for it's camera
					4. Good starting place: [Forum](/home/finn/Documents/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models/iris_with_gimbal/model.sdf) - Use ros-gz packages to map gazebo messages to standard ROS2 image formats, then you can use ROS2 openCV packages to continue
					5. how to [ADD CAMERA TO IRIS_GIMBAL](http://answers.gazebosim.org/question/17662/) sdf want to be able to get a live video feed that I can see in Rviz like [this video ](https://www.youtube.com/watch?v=O2KT_7mCkpA) Then I can use whatever strategy to insert circles into gazebo like he inserted traffic cones
					6. The camera is already in the 3d_gimbal SDF file, which is within the iris_with_standoffs SDF, so no need to input anything.
					7. The issue with the Drone model not appearing in RViz has to do with the [model URI needing to be renamed from model to package](https://github.com/ArduPilot/ardupilot_gz#ardupilot_gz) To fix this; In the Iris_with_standoffs, changed `<uri>model://iris_with_standoffs</uri>` to `<uri>package://ardupilot_gazebo/models/iris_with_standoffs</uri>` and ![[Screenshot from 2025-01-02 17-38-51.png]] to `<include> <uri>package://ardupilot_gazebo/models/gimbal_small_3d</uri> <name>gimbal</name> <pose degrees="true">0 -0.01 -0.124923 90 0 90</pose> </include>`' AND in iris_runway.sdf changed `<uri>model://iris_with_gimbal</uri>` to `<uri>package://ardupilot_gazebo/models/iris_with_gimbal</uri>` 
					8. These changes resulted in the following errors in the terminal output from the launch file `ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true` 
						1. Robot State Publisher Error: `[robot_state_publisher-6] [ERROR] [1735875541.400388909] [sdformat_urdf]: Failed to find sdf canonical link [base_link] [robot_state_publisher-6] Failed to parse robot description using: sdformat_urdf_plugin/SDFormatURDFParser`
						2. Camera Plugin Issues: `[ruby $(which gz) sim-1] [Wrn] [CameraZoomPlugin.cc:218] No scene or camera sensors available.` 
						3. Model Loading Issues: `[ruby $(which gz) sim-2] [GUI] [Wrn] [Model.hh:98] Unable to deserialize sdf::Model`
						4. AI thinks that even though we changed the URI format of the `Iris_with_standoffs`, `iris_with_gimbal` and `iris_runway` the URI somewhere along the chain isnt being updated. But I went through and changed all files in the chain,
						5. periodically check for and kill any hanging process `pkill -f gz-sim`
						6. another claude composer thinks a change of `/home/finn/Documents/ardu_ws/src/ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py` from `pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"` to `pkg_ardupilot_gazebo, "models", "iris_with_standoffs", "model.sdf"` 
							1. This resulted in seeing a black and white outline of the model. Then the AI suggested `source install/setup.bash` which bricked the sim
						7. Also using `ps aux | grep gz`, `ps aux | grep ardupilot`,  `ps aux | grep mavproxy`, and `sudo lsof -i :5760` to check for leftover processes is good practice, if found we should kill individual processes with  `pkill -f gz-sim` and `sudo kill -9 <PID>` and for all `killall -9 gz ardupilot mavproxy.py` (don't forget the PID killing)\
						8. **LEFT OFF** using `ros2 topic list` and `ros2 topic echo ____` we can get data from all topics with the prefix AP, but none of the gazebo topics (including the camera)
						9. Decided I want to backup my files using GIT, following instructions from AI, notes linked here [[VFS Competion/GIT|GIT]] 
							1. Ran into issues initializing sub modules and verifying that the changes I made to my local WS are correct. My ardupilot_gazebo files got wiped from my mistake, re-doing them (with the package thing) then proceeding with git sub module initialization, then backup) 
			1. We found the answer to where to launch a MAVproxy terminal on the [ardupilot_gz github](https://github.com/ArduPilot/ardupilot_gz#ardupilot_gz) 
			2. Now we can move onto re installing the VTOL model and world
				1. Side note on the topics, all topics prefixed with "ap" are published by [ardupilot ROS2 DDS pluggin ](https://github.com/ArduPilot/ardupilot_gz#ardupilot_gz), all other topics are published by gazebo. 
![[Pasted image 20241227104607.png]]