# The erl_visualization package

## Examples:
 + roslaunch erl_visualization test_point_trajectory_display.launch
 + roslaunch erl_visualization test_grid_visualization.launch
 + roslaunch erl_visualization test_grid_visualization.launch model_name:=fla_warehouse1 model_scale:=0.0254
 
 
## Docker usage

1. Make sure docker and docker-compose is installed

        $ sudo apt-get update && sudo apt-get -y install docker
        $ sudo adduser $USER docker
        $ sudo curl -L "https://github.com/docker/compose/releases/download/1.24.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        $ sudo chmod +x /usr/local/bin/docker-compose

2. Log out and log back in to reflect your addition to the docker group.  

3. Open terminal inside docker 

        $ export UID
        $ xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
        $ docker-compose run terminal

4. Load the catkin_ws for erl_visualization and launch visualizations

        $ source /root/code/erl_viz_ws/devel/setup.bash
        $ roslaunch erl_visualization test_point_trajectory_display.launch
        $ roslaunch erl_visualization test_grid_visualization.launch
 
## The headers for display classes in nx_rviz_plugins need to be in the src directory for Qt's automoc to work....

