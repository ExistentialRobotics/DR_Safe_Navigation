#! /bin/bash
rostopic pub /quad0/motors std_msgs/Bool "data: true" &
rostopic pub /quad0/mode std_msgs/Int8 "data: 1" 

