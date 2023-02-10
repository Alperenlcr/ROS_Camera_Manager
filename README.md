# Camera Devices Publisher

The `usb_port_detection.py` script periodically checks for connected camera devices on your computer. If a camera is connected, it is published with a topic name based on its ID. If a camera is disconnected, the corresponding topic is killed. This code serves as an example of a multi-threaded project. 

The `camera_data_pub.py` script is a simple camera publisher that takes the ID as a parameter from the terminal. The `usb_port_detection.py` script calls this publisher.

## Published Topics
The following topics are published:

| Topic Name | Information | Message Type |
| --- | --- | --- |
| /camera | List of published camera IDs | `your_package_name/camera` |
| /cameraX/image_raw | X equals the ID and this topic publishes the raw image data | `sensor_msgs/Image` |

## How to Run
To run the code, first [set](https://medium.com/@lavanyaratnabala/create-custom-message-in-ros-52664c65970d) the custom message type in the repository. Then run the following commands in the terminal:

```
$ roscore
$ python usb_port_detection.py
```

Note: Both scripts should be in the same folder.
