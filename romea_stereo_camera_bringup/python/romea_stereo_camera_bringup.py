# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from romea_camera_bringup import CameraMetaDescription
from romea_camera_bringup import get_sensor_location as get_camera_location
from romea_camera_bringup import get_sensor_configuration as get_camera_configuration
from romea_common_bringup import robot_urdf_prefix, device_namespace
from romea_stereo_camera_description import (
    get_stereo_camera_specifications,
    get_stereo_camera_geometry,
    urdf,
)


class StereoCameraMetaDescription(CameraMetaDescription):
    def __init__(self, meta_description_file_path):
        CameraMetaDescription.__init__(self, meta_description_file_path, "stereo_camera")


def load_meta_description(meta_description_file_path):
    return StereoCameraMetaDescription(meta_description_file_path)


def get_sensor_specifications(meta_description):
    return get_stereo_camera_specifications(
        meta_description.get_type(), meta_description.get_model()
    )


def get_sensor_geometry(meta_description):
    return get_stereo_camera_geometry(meta_description.get_type(), meta_description.get_model())


def get_sensor_location(meta_description):
    return get_camera_location(meta_description)


def get_sensor_configuration(meta_description, ns="configuration"):
    return get_camera_configuration(meta_description)


def urdf_description(robot_namespace, mode, meta_description_file_path):

    meta_description = CameraMetaDescription(meta_description_file_path)

    ros_namespace = device_namespace(
        robot_namespace, meta_description.get_namespace(), meta_description.get_name()
    )

    return urdf(
        robot_urdf_prefix(robot_namespace),
        mode,
        meta_description.get_name(),
        meta_description.get_type(),
        meta_description.get_model(),
        get_sensor_configuration(meta_description),
        get_sensor_location(meta_description),
        ros_namespace,
    )
