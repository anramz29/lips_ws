# Navigation Stack

Trossen Navigation Stack is fraught with problems, With that said, once you can solve the base clock issues within the navigation stack it should be the end of the issues. Please refer to trossen's naviagation stack's [documentation](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros1_packages/navigation_stack_configuration.html)

#### Not an Issue but important to note Important 

When running the navigation stack in the locobot replace `robot_model:=locobot_wx200` with `robot_model:=locobot_wx250s` as seen below. 

```
roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx250s use_lidar:=true rtabmap_args:=-d
```

### Issue #1 

```
[ WARN] [1731885700.701499174]: Could not get transform from locobot/odom to locobot/base_footprint after 0.200000 seconds (for stamp=1731885700.314371)! Error="Could not find a connection between 'locobot/odom' and 'locobot/base_footprint' because they are not part of the same tree.Tf has two or more unconnected trees.. canTransform returned after 0.202066 timeout was 0.2.".
```

From my understanding this issue is caused by the Icreate3 base clock not being sychronized, refer to the Icreate3 [documentation](https://iroboteducation.github.io/create3_docs/setup/compute-ntp) or Lococbot Issues [documentation](
https://docs.trossenrobotics.com/interbotix_xslocobots_docs/troubleshooting.html#less-common-issues). Personally I would first try to











