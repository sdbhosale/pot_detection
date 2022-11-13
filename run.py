## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import argparse
import cv2
import os

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

import pyrealsense2 as rs
import numpy as np
import cv2

def get_intrinsics(rs):
    intr = rs.get_intrinsics()
    intrinsics_mat = np.array([[intr.fx,    0.0, intr.ppx], 
                              [0.0, intr.fy,     intr.ppy], 
                              [0,         0,         1.0]])
    return intrinsics_mat

def get_rotation(pitch):
    pitch = -pitch
    R = np.array([[1,0,0],
                 [0,np.sin(pitch),-np.cos(pitch)],
                 [0,np.cos(pitch),np.sin(pitch)]])
    return R

def get_transformation_matrix(rs):
    K = get_intrinsics(rs)
    theta = 20 * np.pi/180
    height = 0.508
    s = 100
    Ox= 3.2 #-3.2
    Oy = abs(height)/np.tan(theta)
    Wx = 6.40
    Wy = 4.80

    transform_mat = np.zeros((3, 3), dtype=np.float32)
    transform_mat[0, 0] = K[0, 0]
    transform_mat[0, 1] = -K[0, 2] * np.cos(theta)
    transform_mat[0, 2] = s * (K[0, 2] * (height * np.sin(theta) + Oy * np.cos(theta)) - Ox * K[0, 0])
    transform_mat[1, 0] = 0
    transform_mat[1, 1] = K[1, 1] * np.sin(theta) - K[1, 2] * np.cos(theta)
    transform_mat[1, 2] = s * (K[1, 2] * (height * np.sin(theta) + Oy * np.cos(theta)) + K[1, 1] * (height * np.cos(theta) - Oy * np.sin(theta)))
    transform_mat[2, 0] = 0
    transform_mat[2, 1] = -np.cos(theta)
    transform_mat[2, 2] = s * (height * np.sin(theta) + Oy * np.cos(theta))
    print(transform_mat)

    # R = get_rotation(theta)
    # height = -height
    # T = np.array([[1, 0, -Ox], [0, 1, -Oy], [0, 0, -height]], dtype=np.float32) *
    # K[0, 2] = -K[0, 2]
    # K[1, 2] = -K[1, 2]
    # K[1, 1] = -K[1, 1]
    # K[2, 2] = -K[2, 2]
    # print(K @ R @ T)

    # return K @ R @ T
    return transform_mat
    
                      

def run_pipeline(interpreter, args):
    labels = read_label_file(args.labels)
    inference_size = input_size(interpreter)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    
    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    profile = pipeline_profile.get_stream(rs.stream.color)
    M = get_transformation_matrix(profile.as_video_stream_profile())
    print(M @ np.array([0,0,1]))

    # Start streaming
    pipeline.start(config)

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            color_img_resized = cv2.resize(color_image, inference_size)
            run_inference(interpreter, color_img_resized.tobytes())

            # continue
            objs = get_objects(interpreter, args.threshold)[:args.top_k]
            color_image = append_objs_to_img(color_image, inference_size, objs, labels)


            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

            cv2.namedWindow('IPT', cv2.WINDOW_AUTOSIZE)  
        
            cv2.imshow('IPT', cv2.warpPerspective(color_image, M, (640, 480), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP | cv2.WARP_FILL_OUTLIERS))

    finally:

        # Stop streaming
        pipeline.stop()


#find the corners of the object in the image
def find_corners(obj, inference_size):
    height, width = inference_size
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    bbox = obj.bbox.scale(scale_x, scale_y)
    x0, y0 = int(bbox.xmin), int(bbox.ymin)
    x1, y1 = int(bbox.xmax), int(bbox.ymax)
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])

def main():
    default_model_dir = '../all_models'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir,default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default = 0)
    parser.add_argument('--threshold', type=float, default=0.1,
                        help='classifier score threshold')
    args = parser.parse_args()

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()

    run_pipeline(interpreter, args)


def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im

if __name__ == '__main__':
    main()