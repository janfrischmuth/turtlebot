# turtlebot
How To Install Linux, ROS, …
- [ ] open Linux file in vmware fusion
- [ ] set updates: “DON’T update”
- [ ] install VSCode
- [ ] install ROS Melodic
	http://wiki.ros.org/melodic/Installation/Ubuntu
- [ ] install Turtlebot (2)
	http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation
- [ ] in .bashrc add
	# ROS related stuff
	source /opt/ros/melodic/setup.bash
	source ~/turtlebot_ws/devel/setup.bash
	export SVGA_VGPU10=0
- [ ] continue Turtlebot installation here
	https://www.programmersought.com/article/17334218754/
- [ ] in terminal, run JansSkript(1) script 
	Desktop>Uni>Bachelorarbeit>Installation
- [ ] install GeographicLib via 
	sudo apt-get install libgeographic-dev
- [ ] download qualisys, motion_capture_system/simulator
- [ ] create folder for own scripts/repo
- [ ] cd to the directory and in terminal do:
	git init
- [ ] set up git account
	git config —global user.email <email>
	git config --global user.name <name>
	git config --global user.signingkey <key>
- [ ] download own git repo via
	git clone https://github.com/janfrischmuth/turtlebot.git
	git checkout <branch>
- [ ] in terminal install
	sudo apt install python-pip
- [ ] in VSCode install
	Python
	pylint
	ROS
- [ ] to be continued

