#!/usr/bin/env python
import json
import time
import os
import sys
import time
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw

ROOT_DIR = '../Mask_RCNN/'
assert os.path.exists(ROOT_DIR), 'ROOT_DIR does not exist. Did you forget to read the instructions above?'

# Import mrcnn libraries
sys.path.append(ROOT_DIR) 

import mrcnn.model as modellib
import mrcnn.utils as utils
import skimage
from mrcnn import visualize
from mrcnn.config import Config

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Local path to trained weights file
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")

class CocoSynthConfig(Config):
    """Configuration for training on the chess dataset.
    Derives from the base Config class and overrides specific values.
    """
    # Give the configuration a recognizable name
    NAME = "test_chess_cocosynth_dataset"

    # Train on 1 GPU and 1 image per GPU. Batch size is 1 (GPUs * images/GPU).
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

    # Number of classes (including background)
    NUM_CLASSES = 1 + 1  # background + 7 box types

    # All of our training images are 512x512
    IMAGE_MIN_DIM = 512
    IMAGE_MAX_DIM = 512

    # You can experiment with this number to see if it improves training
    STEPS_PER_EPOCH = 1000

    # This is how often validation is run. If you are using too much hard drive space
    # on saved models (in the MODEL_DIR), try making this value larger.
    VALIDATION_STEPS = 5
    
    # Matterport originally used resnet101, but I downsized to fit it on my graphics card
    BACKBONE = 'resnet50'

    # To be honest, I haven't taken the time to figure out what these do
    RPN_ANCHOR_SCALES = (8, 16, 32, 64, 128)
    TRAIN_ROIS_PER_IMAGE = 32
    MAX_GT_INSTANCES = 50 
    POST_NMS_ROIS_INFERENCE = 500 
    POST_NMS_ROIS_TRAINING = 1000 
    
config = CocoSynthConfig()

class InferenceConfig(CocoSynthConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    IMAGE_MIN_DIM = 256
    IMAGE_MAX_DIM = 256
    DETECTION_MIN_CONFIDENCE = 0.90
    

inference_config = InferenceConfig()

# Recreate the model in inference mode
model = modellib.MaskRCNN(mode="inference", 
                          config=inference_config,
                          model_dir=MODEL_DIR)


# ### Load Trained Weights
model_path = str(Path(ROOT_DIR) / "logs" / "cocosynth_dataset20190804T2206/mask_rcnn_cocosynth_dataset_0008.h5" )

# Load trained weights (fill in path to trained weights here)
assert model_path != "", "Provide path to trained weights"
print("Loading weights from ", model_path)
model.load_weights(model_path, by_name=True)

def board_state_detection():
    square_labels = {}
    full_squares = []
    empty_squares = []

    with open('image_paths.json') as json_file:
        square_labels = json.load(json_file)
    
    real_test_dir = 'test/'

    image_paths = []
    for filename in os.listdir(real_test_dir):
        if os.path.splitext(filename)[1].lower() in ['.png', '.jpg', '.jpeg']:
            image_paths.append(os.path.join(real_test_dir, filename))

    # generating the list of empty and full squares based on inference made by model
    for image_path in image_paths:
        img = skimage.io.imread(image_path)
        img_arr = np.array(img)
        results = model.detect([img_arr], verbose=0)
        r = results[0]
        if r["masks"].size == 0:
            empty_squares.append(square_labels[image_path])
        else:
            full_squares.append(square_labels[image_path])

    squares = {
        "empty": empty_squares,
        "full": full_squares
    }

    with open('squares.json', 'w') as fp:
        json.dump(squares,fp, indent=4, sort_keys="True")

def main():
    while True:
        board_state_detection()
        

if __name__ == '__main__':
    main() 


