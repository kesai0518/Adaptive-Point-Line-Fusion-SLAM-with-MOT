import rclpy
from rclpy.node import Node
from slam_interfaces.srv import Semantic 
import cv2

import time
import mmcv
from mmdet.apis import init_detector, inference_detector, show_result_pyplot, show_result_ins, show_result_track

config_file = '/home/kesai/mmdetection-1.0.0/SOLO/configs/solov2/solov2_light_448_r50_fpn_8gpu_3x.py'
# download the checkpoint from model zoo and put it in `checkpoints/`
checkpoint_file = '/home/kesai/mmdetection-1.0.0/SOLO/SOLOv2_LIGHT_448_R50_3x.pth'

# build the model from a config file and a checkpoint file
model = init_detector(config_file, checkpoint_file, device='cuda:0')

# test a single image
#img = '/home/kesai/mmdetection-1.0.0/SOLO/demo/demo.jpg'


class SemanticNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.solo_server = self.create_service(Semantic,"solov2",self.solov2_callback)

    def solov2_callback(self,request,response):
        img = request.picname
        #self.get_logger().info("收到图像，开始处理图像.........")
        start = time.time()
        result = inference_detector(model,img)
        nums, labels, scores, masks, boxes = show_result_track(img, result, model.CLASSES, score_thr=0.4)
        end = time.time()
        print("time cost : ",(end - start))
        # print("num : ", nums)
        # print("the labels is: ",labels)
        # print("the scores is : ", scores)
        # print("the boxes is: ", boxes)
        response.nums = nums
        response.labels = labels
        response.scores = scores
        response.masks = masks
        response.boxes = boxes
        return response


def main(args=None):
    rclpy.init(args=args)
    solov2_node = SemanticNode("solov2_node")
    rclpy.spin(solov2_node)
