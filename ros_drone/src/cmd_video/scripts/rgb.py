#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
import time
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

def if_folder_exists(folder_path):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

i = 0
def resize(img, height=None, width=None):
    """resize the image

    Args:
        img (array): png image
        height (int, optional): pixel. Defaults to None.
        width (int, optional): pixel. Defaults to None.

    Returns:
        image: the raw image
    """
    h, w = img.shape[0], img.shape[1]
    if height:
        width = int(w/h*height)
    else:
        height = int(h/w*width)
    # get the image
    target_img = cv.resize(img, dsize=(width, height))
    return target_img

def save_rgb_data(array, stamp, id):

    folder_path = f"/home/taiic/Desktop/VLN-CE/sample/pic/{stamp}"
    if_folder_exists(folder_path)
    
    file_path = f"{folder_path}/rgb_{id}.npy" 
    np.save(file_path, array)

def convert_depth_image(ros_image, stamp, id):
    bridge = CvBridge()
    global i
    try:

        img = bridge.compressed_imgmsg_to_cv2(ros_image) # compressed
        #img = bridge.imgmsg_to_cv2(ros_image) # raw

        rgb_array = np.array(img, dtype=np.float32)
        
        # # bgr -> rgb
        # b,g,r = cv.split(img)
        # img_rgb = cv.merge([r,g,b])
        
        # test rgb show
        #cv.imshow("cute", img)
        
        # save rgb CompressedImage and data
        idx = str(i).zfill(4)
        # print("Start save rgb CompressedImage and data.")

        save_rgb_data(rgb_array, stamp, id)
        
        folder_path2 = f"/home/taiic/Desktop/VLN-CE/sample/pic/{stamp}"
        if_folder_exists(folder_path2)

        cv.imwrite(f"{folder_path2}/rgb_{id}.png", img)
        # print("Saved successfully.")
        #i += 1
        cv.waitKey(1)
    except CvBridgeError as e:
        print(e)
        
def pixel2rgb():
    rospy.init_node("lzc",anonymous=True)
    
    rgb_time = rospy.get_param("/listen_rgb/rgb_stamp")
    rgb_idx = rospy.get_param("/listen_rgb/rgb_id")

    # always
    # rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, callback=convert_depth_image, queue_size=1)
    # rospy.spin()
    
    # just once
    compressed_image = rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage)
    convert_depth_image(compressed_image, rgb_time, rgb_idx)
    
if __name__ == "__main__":
    start_ = time.time()
    pixel2rgb()
    end_ = time.time()
    print(f"Saved rgb image cost time: {end_ - start_}.")