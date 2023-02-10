#!/usr/bin/python3
"""
Detects usb cameras and sends them with publisher
"""
from time import sleep
import os
import subprocess
import threading
import rospy
from std_msgs.msg import Int64MultiArray
from ui_messages.msg import camera


class CameraLauncher:
    """Camera launching functions"""

    def __init__(self):
        self.pids = {}
        self.path = os.path.dirname(__file__) + "/"

    def terminate(self, camera_id):
        """Terminates the camera"""
        os.system("kill " + str(self.pids[camera_id]))
        self.pids.pop(camera_id)
        print("Camera " + str(camera_id) + " is shutdowned")

    def run(self, camera_id):
        """Runs the camera publisher"""
        if self.path == "/":
            self.path = ""
        process = subprocess.Popen(
            ["python3", self.path + "camera_data_pub.py", str(camera_id)]
        )
        self.pids[camera_id] = process.pid


def camera_ids_pub(camera_list: list):
    """IDs of connected cameras publisher"""
    camera_publisher = rospy.Publisher("camera", camera, queue_size=10)
    # Create a new SensorInformation object and fill in its contents.
    camera_info = camera()
    # Fill in the header information.
    camera_info.id = len(camera_list)
    camera_info.cameras = Int64MultiArray(data=camera_list)
    camera_publisher.publish(camera_info)


def camera_thread():
    """Runs the camera publisher"""
    rospy.init_node("roverCameraNode", disable_signals=True)
    while True:
        camera_ids_pub(camera_ids)


def check_devices():
    """Checks USB connected devices"""
    temp = os.popen("v4l2-ctl --list-devices").readlines()
    usb_devices = []
    flag = 1
    temp1 = []
    for i in temp:
        if "video" in i and flag == 1:
            temp1.append(i[-2])
            flag = 0
            temp1.append(i[-2])
            usb_devices.append(temp1)
            temp1 = []
        elif "video" not in i and i != "\n":
            temp1.append(i[:-2])
            flag = 1
    if len(usb_devices) == 0:
        return usb_devices, []

    # spectrum kamerasini calistirmamak icin filtreleme
    for i in usb_devices:
        if "Microsoft" in i[0] and "LifeCam" in i[0]:
            usb_devices.remove(i)  # pylint: disable=modified-iterating-list
            break

    return usb_devices, sorted(list(map(int, list(zip(*usb_devices))[2])))


def init():
    """Runs only once at start"""
    global THREADS  # pylint: disable=global-statement global-variable-not-assigned
    global SERVER_PID  # pylint: disable=global-statement
    first_devices, first_camera_ids = check_devices()
    sleep(2)
    for index in first_camera_ids:
        THREADS[int(index)] = threading.Thread(
            target=camera_launcher.run, args=(int(index),)
        )
    for index in first_camera_ids:
        list(THREADS.values())[list(THREADS.keys()).index(int(index))].start()
    camera_thread_id = threading.Thread(target=camera_thread)
    camera_thread_id.start()
    return first_devices, first_camera_ids


# Main Code
camera_launcher = CameraLauncher()
SERVER_PID = 0
THREADS = {}
old_devices, old_camera_ids = init()
while True:
    devices, camera_ids = check_devices()
    print("\n------------")
    print("DEVICES:")
    print(devices)
    print("CAMERA IDs:")
    print(camera_ids)
    sleep(2)

    if camera_ids != old_camera_ids:
        news = list(set(camera_ids) - set(old_camera_ids))
        olds = list(set(old_camera_ids) - set(camera_ids))
        if news:
            print(news, " are new, open them.")
            for number in news:
                THREADS[int(number)] = threading.Thread(
                    target=camera_launcher.run, args=(int(number),)
                )
            for number in news:
                list(THREADS.values())[list(THREADS.keys()).index(int(number))].start()

        if olds:
            print(olds, " are unplugged, close them.")
            for number in olds:
                camera_launcher.terminate(int(number))
                THREADS[int(number)].join()
                THREADS.pop(int(number))

    old_devices, old_camera_ids = devices, camera_ids
