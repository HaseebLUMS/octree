#!/usr/bin/env python3.6

import pclpy
from pclpy import pcl
import numpy as np
import cv2
from skimage.metrics import structural_similarity as ssim
import matplotlib.pyplot as plt

from queue import Queue, SimpleQueue


def load_point_cloud(file_path):
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PLYReader()
    reader.read(file_path, cloud)
    return cloud

def set_camera_view(viewer, pos, view, up):
    viewer.setCameraPosition(pos[0], pos[1], pos[2],
                             view[0], view[1], view[2],
                             up[0], up[1], up[2])

def render_point_cloud(cloud, camera_position, image_size=(640, 480)):
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.addPointCloud(cloud)

    set_camera_view(viewer, *camera_position)

    viewer.setSize(image_size[0], image_size[1])
    viewer.setBackgroundColor(1, 1, 1)
    
    img = np.zeros((image_size[1], image_size[0], 3), dtype=np.uint8)
    viewer.setScreenshotPath("output/screenshot.png")
    viewer.spinOnce(10, True)
    viewer.saveScreenshot("output/screenshot.png")
    
    img = cv2.imread("output/screenshot.png")
    viewer.close()
    return img

def compute_ssim(imageA, imageB):
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)
    score, diff = ssim(grayA, grayB, full=True)
    return score

# Load the point cloud files
cloud1 = load_point_cloud('output/ricardo.ply')
cloud2 = load_point_cloud('output/ricardo.ply')

# Define camera position (pos, view, up)
camera_position = ([0, 0, 10], [0, 0, 0], [0, 1, 0])

# Render the point clouds and capture images
image1 = render_point_cloud(cloud1, camera_position)
image2 = render_point_cloud(cloud2, camera_position)

# Compute SSIM between the images
ssim_score = compute_ssim(image1, image2)

print(f'SSIM Score: {ssim_score}')