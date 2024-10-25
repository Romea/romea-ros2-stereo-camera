# romea_stereo_camera_bringup #

# 1) Overview #

The romea_stereo_camera_bringup package provides  : 

 - **Launch files** able to launch ros2 camera drivers according a meta-description file provided by user (see next section for camera meta-description file overview), supported drivers are :

   - [???](TODO(Jean))
   - [???](TODO(Jean))

   To launch a driver via command line, use:

    ```console
    ros2 launch romea_stereo_camera_bringup camera_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - A **Python module** able to load and parse stereo camera meta-description file as well as to create URDF description of the stereo camera according a given meta-description.

 - A **ros2 python executable** able to create stereo camera URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_stereo_camera_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > camera.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and other sensor URDFs to create a complete URDF description of the robot.  

   

# 2) Stereo camera meta-description #

The stereo camera meta-description file is a YAML file with the following five main items:
- name: Specifies the name of the camera.
- driver: Specifies the configuration for the ROS2 driver used to control the stereo camera (see Section 5 for more details).
- configuration: Defines basic specifications of the camera, such as type, model, resolution, field of view, frame rate, and video format.
- geometry: Defines the position and orientation of the robot, with its position and orientation, used for URDF creation.
- records: Specifies the topics that will be recorded in ROS bags during experiments or simulations. Remappings ensure the GPS topics have consistent names across drivers and simulation.

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

The supported camera models are listed below:

|  type  |   model    |
| :----: | :--------: |
| zed    |     1      |
| zed    |     2      |

For detailed specifications, refer to the config directory within the romea_camera_description package.

# 5) Supported stereo camera ROS2 drivers

TODO(Jean)