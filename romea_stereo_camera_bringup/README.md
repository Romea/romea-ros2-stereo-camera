# 1) Overview #

The romea_stereo_camera_bringup package provides  : 

 - launch files able to launch ros2 camera drivers according a meta-description file provided by user (see next section for camera meta-description file overview), supported drivers are :

   - [???](TODO(Jean))
   - [???](TODO(Jean))

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_stereo_camera_bringup camera_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - a python module able to load and parse stereo camera meta-description file as well as to create URDF description of the stereo camera according a given meta-description.

 - a ros2 python executable able to create stereo camera URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_stereo_camera_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > camera.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and other sensor URDFs to create a complete URDF description of the robot.  

   

# 2) Stereo camera meta-description #

As seen below stereocamera meta-description file is a yaml file constituted by five items. The first item is the name of sensor defined by user. The second one is the configuration of ROS2 driver used to control stereo camera (see section 4 for more explanations). The third item provides basics specifications of the camera and the fourth item specifies where the camera is located on the robot, these informations will be used to create URDF description and by user to configure its algorithms.  Finally, the last item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, stereo camera topics are always the same names for each drivers or simulator plugins.       

Example :
```yaml
  name: "stereo_camera"  # name of the camera given by user
  driver: # stereo camera driver configuration
    pkg: TODO(Jean)  
 configuration: # camera basic specifications
    type: zed  #  type of camera
    model: "1"  # model of camera
    resolution: 1280x720 # resolution of image provided by camera 
    horizontal_fov: 85.0 # horiizontal field of view (optional)
    frame_rate: 30 # frame rate in hz (optional)
    video_format: h264 # output video codec (optional)
geometry: # geometry configuration 
  parent_link: "base_link"  # name of parent link where is located the camera
  xyz: [1.02, 0.0, 1.34] # its position in meters
  rpy: [0.0, 0.2, 0.0] # its orienation in degrees
records: # topic to be recorded
  left/image_raw: true # left raw image will be recorded into bag 
  left/camera_info: false # left camera info will not be recorded into bag
  right/image_raw: true # right raw image will be recorded into bag 
  right/camera_info: false # right camera info will not be recorded into bag
```

# 4) Supported  stereo camera models

Supported stereo camera are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| zed    |     1      |
| zed    |     2      |

You can find specifications of each camera in config directory of romea_camera_description package.

# 5) Supported stereo camera ROS2 drivers

TODO(Jean)