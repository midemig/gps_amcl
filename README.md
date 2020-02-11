GPS AMCL
=============================

The gps_amcl ROS package implements a modified version of original amcl node from navigation package (https://github.com/ros-planning/navigation) that improves autonomous vehicles localization using a modification of probabilistic laser localization like Monte Carlo Localization (MCL) algorithm, enhancing the weights of the particles by adding kalman filtered GNSS information. GNSS data are used to improve localization accuracy in places with fewer map features and to prevent the kidnapped robot problems. Besides, Laser information improves accuracy in places where the map has more features and GNSS higher covariance, allowing the approach to be used in specifically difficut scenarios for GNSS such as urban canyons.
The algorithm is tested using KITTI odometry dataset proving that it improves localization compared with classic GNSS+INS fusion and AMCL.


### Example of usage 


	Real-driving data can be obtained from http://www.cvlibs.net/datasets/kitti/raw_data.php?type=residential and converted into bagfile using kitti2bag from https://github.com/tomas789/kitti2bag (/tf and static_tf topics must be filtered)

    roslaunch gps_amcl gps_amcl.launch
    rosbag play kitti kitti_2011_09_26_drive_0036_synced.bag --clock


### Videos

 * *GPS AMCL*

![alt-text](demo_videos/gps_amcl.gif)


 * *ORIGINAL AMCL*

![alt-text](demo_videos/amcl.gif)





### Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install gps_amcl
