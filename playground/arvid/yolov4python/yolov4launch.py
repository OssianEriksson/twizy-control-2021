import cv2

from yolov4.tf import YOLOv4

yolo = YOLOv4()

yolo.config.parse_names("coco.names")
yolo.config.parse_cfg("yolov4.cfg")

yolo.make_model()
yolo.load_weights("yolov4.weights", weights_type="yolo")
yolo.summary(summary_type="yolo")
yolo.summary()

#yolo.inference(media_path="download.jpeg")

#yolo.inference(media_path="road.mp4", is_image=False)

yolo.inference(
   "/dev/video0",
   is_image=False,
   cv_apiPreference=cv2.CAP_V4L2,
   cv_frame_size=(640, 480),
   cv_fourcc="YUYV",
)