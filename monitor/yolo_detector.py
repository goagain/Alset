import cv2
import torch
import time
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages, letterbox
from utils.general import check_img_size, check_requirements, non_max_suppression, apply_classifier, scale_coords, \
    xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized
import TLState
import detectColor  

class yolo_detector:
    def __init__(self, img_size=640, stride=32, debug = 0):
        path = 'foo'
        opt_device = '0'
        weights = 'yolov5s.pt'
        device = select_device(opt_device)
        print(f'PyTorch selected device = {device}')
        half = device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        model = attempt_load(weights, map_location=device)  # load FP32 model
        stride = int(model.stride.max())  # model stride
        imgsz = check_img_size(img_size, s=stride)  # check img_size
        if half:
            model.half()  # to FP16

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names
        print(names)
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]
        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        t0 = time.time()
        self.model = model
        self.img_sz = imgsz
        self.device = device
        self.half = half
        self.names = names
        self.colors = colors
        self.stride = stride
        self.debug = debug
        cv2.namedWindow('detect', cv2.WINDOW_NORMAL)

    def detect(self, image, frame=12345):
        if self.debug>0:
            print(type(image))
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        img_np = array[:, :, :3]
        opt_augment = False
        opt_agnostic_nms = False
        opt_classes = None
        opt_iou_thres = 0.45
        opt_conf_thres = 0.25
        opt_save_conf = False
        newimg_np = letterbox(img_np, self.img_sz, stride=self.stride)[0]
        bgr_image = cv2.cvtColor(newimg_np, cv2.COLOR_RGB2BGR)
        im0s = bgr_image
        if self.debug>0:
            print(f'im0s shape:',im0s.shape)
        img = im0s[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        if self.debug>0:
            print(f'img shape:',img.shape)
        img = np.ascontiguousarray(img)
        if self.debug>0:
            print(f'mewimg shape:',img.shape)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img, augment=opt_augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt_conf_thres, opt_iou_thres, classes=opt_classes, agnostic=opt_agnostic_nms)
        t2 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            s = ''
            gn = torch.tensor(im0s.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0s.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    line = (cls, *xywh, conf) if opt_save_conf else (cls, *xywh)  # label format
                    if self.debug>0:
                        print('line:',line)

                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    if self.debug>0:
                        print('label:',label)
                    if label.find('traffic light')>=0:
                        traffic_img = newimg_np[int(xyxy[1]):int(xyxy[3]),int(xyxy[0]):int(xyxy[2]),:]
                        # print('original size:',newimg_np.shape)
                        # print('box data:',xyxy)
                        # print('traffic_img shape:',traffic_img.shape)
                        cv2.imshow('detect', traffic_img)
                        cv2.waitKey(1) # 1 millisecond
                        if traffic_img.shape[0]>0:
                            state = TLState.detectState(traffic_img,TLState.TLType.regular.value)
                            label_new = label
                            if state>0:
                                label_new = TLState.TLState(state).name
                            # else:
                            #     label_new = 'unknown'
                            label = label + " - " + label_new
                    plot_one_box(xyxy, newimg_np, color=tuple(self.colors[int(cls)]), label=label, line_thickness=1)

            # Print time (inference + NMS)
            if self.debug>0:
                print(f'{s}Done. ({t2 - t1:.3f}s)')

            # cv2.imshow('detect', newimg_np)
            # cv2.waitKey(1) # 1 millisecond
            bgr_image = cv2.cvtColor(newimg_np, cv2.COLOR_RGB2BGR)
            return bgr_image