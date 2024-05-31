# README

This is an updated version of Ben Charrow's scarab repo: https://github.com/bcharrow/scarab


# Scarab Setup Instructions
##1. WiFi communications to scarab from host laptop.
1. Connect to the WiFi network called "scarab" (the p/w is mrsl_12345)
2. In your browser, enter 192.168.131.1
3. Connect to router (user is admin and p/w is password)
4. Navigate to Advanced tab, then Setup, then Lan Setup, and then scroll to the bottom of the page to add your machine.  In the page that follows, your machine should appear.  Click add.
5. Open a terminal and run: ```ssh root@192.168.131.44```. _Note that the last digits (here 44) refer to the scarab ID at hand.  Please use the appropriate ID number if working with another vehicle._
6. Once ssh'd to scarab, run in the terminal: ```adduser drlp```. _Note that the user already exists for scarab 44, so this step can be skipped._
7. Add your machine's IP to the etc/hosts file in scarab.  In the same terminal (the one ssh'd to scarab) run ```<your favorite text editor> /etc/hosts```, and add the following line in the file that opens ```192.168.131.155 <your machine name>```.

##2. Setup ROS in scarab 
First, setup your ROS workspace in scarab by running the following commands in terminal (in scarab of course!):

1. mkdir /home/drlp/catkin_ws
2. mkdir /home/drlp/catkin_ws/src
3. cd /home/drlp/catkin_ws/src && catkin_init_workspace

Then source the workspace by adding the following code in scarab's ~/.bashrc:
```
function init_ros()
{
  source /opt/ros/indigo/setup.bash
  WORKDIR=/home/drlp/catkin_ws 
  source $WORKDIR/devel/setup.bash 
  echo "Sourcing $WORKDIR/devel/setup.bash"
}
```
Finally, add the following code to call the above function (again in scarab's ~/.bashrc):
```
init_ros
```

##3. ROS communications setup between host computer and scarab
First, add the following code **_both_** in your machine's and scarab's ~/.bashrc:
``` 
function init_scarab()
{
  MYIP=`ifconfig | grep 'inet addr:'| grep -v '127.0.0.1'| grep 'inet addr:'| cut -d: -f2 | awk '{print $1}' | egrep '(192.168.131)'`
  
  if [ "$1" != "" ] ; then
    export ROS_MASTER_URI=http://192.168.131.$1:11311;
	else
	  export ROS_MASTER_URI=http://$MYIP:11311;
	fi
  export ROS_IP=$MYIP
	echo "ROS Communication setup to use scarab with my computer!  Scarab shall be the ROS Master."
	echo "ROS_IP: $ROS_IP, ROS_MASTER_URI: $ROS_MASTER_URI"  
}
```

Then add the following lines in your machine's and scarab's ~/.bashrc to call the above function:
```
init_scarab      (laptop)
init_scarab 100  (scarab)
```
where 100 refers to the last digits of your IP address.

##4. Transfer code from host laptop to scarab
Open a terminal in your host machine and run:
```
rsync -avz --exclude="*.bag" --inplace scarab drlp@192.168.131.44:/home/drlp/catkin_ws/src
```

##5. Human Friendly Navigation

1. Turn on scarab and roboclaw 
2. ssh into the scarab and make sure the ROS_MASTER_URI is correct
3. (laptop) ```roscore```
4. (scarab) ```roslaunch scarab scarab.launch```
5. (scarab) ```roslaunch scarab gmapping.launch```
6. (scarab) ```roslaunch scarab hfn.launch```
7. (laptop) ```rosrun rviz rviz -d $(rospack find scarab)/scarab.rviz``` (or possibly scarab_hfn_experiment.rviz)
8. (GO) ```rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'```
9. (STOP) ```rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'```


+ Pose topic: ```/odom_laser``` type: ```nav_msgs/Odometry```
+ Laser topic: ```/scan``` type: ```sensor_msgs/LaserScan```
+ Command topic: ```/cmd_vel``` type: ```geometry_msgs/Twist```

##New cleaned up code for launching hfn

1. (laptop) ```roscore```
2. (scarab) ```roslaunch scarab_launch scarab_bringup.launch```
3. (scarab) ```roslaunch scarab_launch scarab_mapping.launch```
4. (scarab) ```roslaunch scarab_launch scarab_hfn.launch```
5. (laptop) ```rosrun rviz rviz -d $(rospack find scarab_launch)/rviz_cfg/scarab44_cfg.rviz```


##6. Recording a rosbag

0. ```rosbag record -a```
1. ```rosbag record -O subset.bag /scan /odom_laser /map_hokuyo /goal /cmd_vel /laser_cloud /odom_motor /scan_throttle /tf /tf_static```
2. ```rosbag play --pause -s 2 -r 2 subset.bag```
3. ```rosbag info subset.bag```

## TODO:
- What does roboclaw do? What are the PID parameters?
- We want to add the correct dependencies to the scarab meta-package.
- How to e-stop scarab? (See STOP command above)

## Known issues:
- If there are issues in rviz with the laser scan having errors (blinking) this is likely a network access issue. Moving the laptop communicating with the scarab to another location solved this issue before.
