"""
This file contains all the methods responsible for saving the generated data in the correct output format.

"""

import numpy as np
import os
import logging
import json

def read_json_file(filename):
    return [json.loads(line) for line in open(filename, 'r')]

def save_obstacle_detector_data(filename, data):
    with open(filename, 'a') as f:
        f.write(json.dumps(data)+'\n')
    #logging.info("Wrote obstacle_detector_data to %s", filename)

def save_velocity_data(filename, data):
    with open(filename, 'a') as f:
        f.write(json.dumps(data)+'\n')
    #logging.info("Wrote velocity_data to %s", filename)

def save_image_data(filename, image, mode):
    #logging.info("Wrote image data to %s", filename)
    image.save_to_disk(filename, mode)
