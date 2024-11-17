## Odometry Issues

#### This has been a long standing issue, there are a couple of possible solutions
<img src="images/Rviz_Odom_Issue.png" alt="Alt text" width="1200" height="600">

#### Quick solution
I found this [solution](https://github.com/Interbotix/interbotix_ros_rovers/issues/25) on trossen's github issues page linked below. If all you need to do is go into rviz, or you don't have time to go over the long solution below you can always get the robot figure in rviz. Note that this messes up the base odometry and therefore you aren't able to move the robot with the navigation or joystick packages, (at least that's my expirence).



```bash
roslaunch interbotix_xslocobot_descriptions remote_view.launch rviz_frame:=locobot/base_link
```

#### Once you re-run rviz you, the rviz_frame should look like this
<img src="images/Rviz_Issue_Semi_Solved.png" alt="Alt text" width="1200" height="600">