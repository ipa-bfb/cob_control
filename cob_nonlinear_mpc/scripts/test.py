import pyros_setup
try:
    import rospy
    import roslaunch
    import rosgraph
    import rosnode
except ImportError:  # if ROS environment is not setup, we emulate it.
    pyros_setup.configurable_import().configure('mysetup.cfg').activate()  # this will use mysetup.cfg from pyros-setup instance folder
    import rospy
    import roslaunch
    import rosgraph
    import rosnode

print('hello')