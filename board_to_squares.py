#!/usr/bin/env python

"""

This node takes a full image of the chess game, and creates 64 seperate images. (One image per square of the board). 
It is using Canny Edge detection and Hough Transform to detect the chess board in the image and get its four corners.
After that the image gets cropped to the chessboard and 64 images are created from the cropped image (one for each square).

"""

import cv2
import numpy as np
import json
import scipy.spatial as spatial
import scipy.cluster as clstr
from collections import defaultdict
from functools import partial
from PIL import Image
from measurements import square_label
from measurements import squares_integers
import time

SQUARE_SIDE_LENGTH = 227


def split_board(img):
    """
    Given a board image, returns an array of 64 smaller images.
    """
    arr = []
    sq_len = img.shape[0] / 8
    for i in range(8):
        for j in range(8):
            arr.append(img[i * sq_len : (i + 1) * sq_len, j * sq_len : (j + 1) * sq_len])
    return arr

def create_squares():
    img = cv2.imread('cropped.jpg')
    arr = split_board(img)
    image_path_dictionary = {}
    for i in range(len(arr)):
        # Path to the image   
        img_path = 'test/' + str(i) + '.jpg'
        cv2.imwrite(img_path, arr[i])
        image_path_dictionary[img_path] = squares_integers[square_label[i].lower()]
    with open('image_paths.json', 'w') as fp:
        # Create a json file which will map each image to its respective integer according to the python-chess module
        json.dump(image_path_dictionary,fp, indent=4, sort_keys="True")

def main():
    while True:
        create_squares()
        time.sleep(5)

if __name__ == '__main__':
    main()



