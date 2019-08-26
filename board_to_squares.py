#!/usr/bin/env python

"""
This nodes takes a full image of the chess game, and creates 64 seperate images. (One image per square of the board). 
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


square_label = [
    "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8",
 "B1", "B2", "B3", "B4", "B5", "B6", "B7","B8",
 "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8",
 "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8",
 "E1", "E2", "E3", "E4", "E5", "E6", "E7", "E8",
 "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8",
 "G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8",
 "H1", "H2", "H3", "H4", "H5", "H6", "H7", "H8"
 ]

squares_integers = {
    'a1': 0,
    'b1': 1,
    'c1': 2,
    'd1': 3,
    'e1': 4,
    'f1': 5,
    'g1': 6,
    'h1': 7,
    'a2': 8,
    'b2': 9,
    'c2': 10,
    'd2': 11,
    'e2': 12,
    'f2': 13,
    'g2': 14,
    'h2': 15,
    'a3': 16,
    'b3': 17,
    'c3': 18,
    'd3': 19,
    'e3': 20,
    'f3': 21,
    'g3': 22,
    'h3': 23,
    'a4': 24,
    'b4': 25,
    'c4': 26,
    'd4': 27,
    'e4': 28,
    'f4': 29,
    'g4': 30,
    'h4': 31,
    'a5': 32,
    'b5': 33,
    'c5': 34,
    'd5': 35,
    'e5': 36,
    'f5': 37,
    'g5': 38,
    'h5': 39,
    'a6': 40,
    'b6': 41,
    'c6': 42,
    'd6': 43,
    'e6': 44,
    'f6': 45,
    'g6': 46,
    'h6': 47,
    'a7': 48,
    'b7': 49,
    'c7': 50,
    'd7': 51,
    'e7': 52,
    'f7': 53,
    'g7': 54,
    'h7': 55,
    'a8': 56,
    'b8': 57,
    'c8': 58,
    'd8': 59,
    'e8': 60,
    'f8': 61,
    'g8': 62,
    'h8': 63
    }

def create_squares():
    # Takes cropped image genera
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



