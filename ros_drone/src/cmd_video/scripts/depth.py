#!/usr/bin/env python
import rospy
import numpy as np
import time
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from PIL import Image as pi

def if_folder_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

def save_depth_data(array, stamp, id):

    folder_path = f"/home/taiic/Desktop/VLN-CE/sample/pic/{stamp}"
    if_folder_exists(folder_path)
    file_path = f"{folder_path}/depth_{id}.npy"
    np.save(file_path, array)
    # np.savetxt(file_path+"11",array)


def convert_depth_image(ros_image, stamp, id):
    bridge = CvBridge()
    #global i
    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough") # raw
        #depth_image = bridge.compressed_imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        # print(depth_image[0])
        depth_array = np.array(depth_image, dtype=np.float32)
        # print(depth_array.shape)
        # print(depth_array[0])
        
        # get one pixel
        center_idx = np.array(depth_array.shape) / 2
        # print("center depth:",depth_array[240,320])

        im = pi.fromarray(depth_array)
        im = im.convert("L")
        
        # print("Begin save depth image and data.")
        # save image and data
        ## save data
        save_depth_data(depth_array, stamp, id)
        ## save depth image
        #im.save(f"/home/taiic/Desktop/VLN-CE/sample/{time_stamp}.png")
        # print("Saved successfully.")
        
    except CvBridgeError as e:
        print(e)
        
def pixel2depth():
    rospy.init_node("lzh",anonymous=True)
    
    depth_time = rospy.get_param("/listen_depth/depth_stamp")
    depth_idx = rospy.get_param("/listen_depth/depth_id")
    
    # keep
    # rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback=convert_depth_image, queue_size=1)
    # rospy.spin()
    
    # # only once
    compressed_depth_image = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image)
    convert_depth_image(compressed_depth_image, depth_time, depth_idx)
    
if __name__ == "__main__":
    start_ = time.time()
    pixel2depth()
    end_ = time.time()
    print(f"Saved depth image cost time: {end_ - start_}.")