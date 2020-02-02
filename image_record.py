#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from PIL import Image
import time
    
def get_image():
    rospy.init_node('image_record', anonymous=True)
    rospy.loginfo("Getting image...")
    image_msg = rospy.wait_for_message("/usb_cam/image_raw/compressed", CompressedImage)
    rospy.loginfo("Got image!")

    # Image to numpy array
    np_arr = np.fromstring(image_msg.data, np.uint8)

    # Decode cv2 image and store
    cv2_img = cv2.imdecode(np_arr, 1)
    img_file_path = "wow2.jpg"
    cv2.imwrite(img_file_path, cv2_img)
    rospy.loginfo("Saved to: " + img_file_path)
    return img_file_path

def crop_image():
    # The image is then opened and cropped so it approximately corresponds to the chessboard
    imageObject = Image.open("./wow2.jpg")
    
    # Crop the chess portion
    cropped = imageObject.crop((205,25,625,445))
    # Rotating image by 90 degrees to have correct square labels
    transposed =  cropped.transpose(Image.ROTATE_90)
    transposed.save("cropped.jpg")

def main():
   while True:
        get_image()
        crop_image()
        time.sleep(5)

if __name__ == '__main__':
    main()
    
