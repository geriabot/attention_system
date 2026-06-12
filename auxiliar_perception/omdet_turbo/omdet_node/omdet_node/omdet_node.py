import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from omdet_node_msgs.srv import SetDetectionPrompt

import cv2
from cv_bridge import CvBridge

from pathlib import Path
from boxmot.trackers import StrongSort

import time

import numpy as np

import torch
from transformers import AutoProcessor, OmDetTurboForObjectDetection

POST_PROCESS_THRESHOLD = 0.3
NMS_THRESHOLD = 0.3

MAX_OUTPUT_HEIGHT = 900
MAX_OUTPUT_WIDTH = 1000

IMG_SUB_QOS = QoSProfile(
  depth=1,
  reliability=ReliabilityPolicy.BEST_EFFORT,
  durability=DurabilityPolicy.VOLATILE
)
DET_PUB_QOS = 10
IMG_PUB_QOS = 10

class OmdetNode(LifecycleNode):
  def __init__(self):
    super().__init__('omdet_node')

    self.declare_parameter('classes', [])
    self.declare_parameter('task', '')
    self.declare_parameter('debug', False)
    self.declare_parameter('image_topic', 'image_rgb')

    self.declare_parameter('tracking', False)
    self.declare_parameter('max_age',       30)
    self.declare_parameter('min_hits',       1)
    self.declare_parameter('iou_threshold',  0.3)
    self.declare_parameter('min_conf',       0.3)
    self.declare_parameter('max_cos_dist',   0.2)
    self.declare_parameter('max_iou_dist',   0.7)
    self.declare_parameter('nn_budget',      100)
    self.declare_parameter('mc_lambda',      0.98)
    self.declare_parameter('ema_alpha',      0.9)
    self.declare_parameter('reid_weights',  '')
    self.declare_parameter('device',        '')

    self._classes = []
    self._task = None
    self._debug = False
    self._tracking = False
    self._img_topic = 'image_rgb'
    self._model_device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    self._bridge = None
    self._processor = None
    self._model = None
    self._tracker = None
    self._detections_pub = None
    self._image_pub = None
    self._in_img_sub = None
    self._class_sub = None
    self._prompt_srv = None

  def on_activate(self, previous_state: State):
    self.get_logger().info("Activating OMDET node")

    # --- Parameters ---
    self._img_topic = self.get_parameter('image_topic').get_parameter_value().string_value

    classes_param = self.get_parameter('classes')
    self._classes = list(classes_param.get_parameter_value().string_array_value)
    self.get_logger().info(f"Classes: {self._classes}")
    
    task_param = self.get_parameter('task').get_parameter_value().string_value
    if task_param == '':
      self._task = None
      self.get_logger().info("No task set.")
    else:
      self._task = task_param
      self.get_logger().info(f"Task: {self._task}")
        
    self._debug = self.get_parameter('debug').get_parameter_value().bool_value
    self.get_logger().info(f"Debug: {self._debug}")

    self._tracking = self.get_parameter('tracking').get_parameter_value().bool_value
    self.get_logger().info(f"Tracking: {self._tracking}")

    # --- OpenCV ---
    self._bridge = CvBridge()

    # --- Omdet ---
    t0 = time.time()
    self._processor = AutoProcessor.from_pretrained("omlab/omdet-turbo-swin-tiny-hf", use_fast=True)
    t_diff = time.time() - t0
    self.get_logger().info(f"Processor import time: {t_diff}")

    t0 = time.time()
    self._model = OmDetTurboForObjectDetection.from_pretrained("omlab/omdet-turbo-swin-tiny-hf")
    t_diff = time.time() - t0 
    self.get_logger().info(f"Model import time: {t_diff}")

    self._model = self._model.to(self._model_device)
    self.get_logger().info(f"Setting model to {self._model_device}")

    # --- Tracking ---
    if self._tracking:
      try:
        p = self._read_tracking_parameters()
        if p is None:
          return TransitionCallbackReturn.ERROR

        device = self._resolve_device(p['device'])
        reid_weights = self._resolve_weights(p['reid_weights'])

        self.get_logger().info(f"Iniciando StrongSORT en {device} con pesos {reid_weights}...")

        self._tracker = StrongSort(
          reid_weights=reid_weights,
          device=device,
          half=False,
          min_conf=p['min_conf'],
          max_cos_dist=p['max_cos_dist'],
          max_iou_dist=p['max_iou_dist'],
          max_age=p['max_age'],
          n_init=p['min_hits'],
          nn_budget=p['nn_budget'],
          mc_lambda=p['mc_lambda'],
          ema_alpha=p['ema_alpha']
        )
        self.get_logger().info("Tracking initialized")
      except Exception as e:
        self.get_logger().error(f"FALLO CRÍTICO EN TRACKER: {str(e)}")
        import traceback
        self.get_logger().error(traceback.format_exc())
        return TransitionCallbackReturn.ERROR

    # --- Publishers ---
    self._detections_pub = self.create_lifecycle_publisher(
      Detection2DArray,
      "/detections_2d",
      DET_PUB_QOS
    )

    self._image_pub = self.create_lifecycle_publisher(
      Image,
      "/omdet_dbg",
      IMG_PUB_QOS
    )

    # --- Subscribers & services ---
    self.get_logger().info(f"Img topic: {self._img_topic}")
    self._in_img_sub = self.create_subscription(
      Image,
      self._img_topic,
      self._image_callback,
      IMG_SUB_QOS
    )

    self._prompt_srv = self.create_service(
      SetDetectionPrompt,
      'omdet_prompt',
      self._set_detection_prompt_callback
    )

    parent_result = super().on_activate(previous_state)
    if parent_result != TransitionCallbackReturn.SUCCESS:
      self.get_logger().error("Failed to activate lifecycle publishers")
      return parent_result

    self.get_logger().info("Returning success")

    self.get_logger().info("Returning success")

    return TransitionCallbackReturn.SUCCESS

  def on_deactivate(self, previous_state: State):
    parent_result = super().on_deactivate(previous_state)
    if parent_result != TransitionCallbackReturn.SUCCESS:
      self.get_logger().error("Failed to deactivate lifecycle publishers")
      return parent_result

    self._destroy_runtime_interfaces()
    self._release_resources()

    self.get_logger().info("OMDET node deactivated")

    return TransitionCallbackReturn.SUCCESS

  def _set_detection_prompt_callback(self, request, response):
    self._update_classes(request.prompt)
    return response

  def _update_classes(self, prompt: str):
    self._classes = [prompt]
    self.get_logger().info(f'New class to detect: {prompt}')

  def _destroy_runtime_interfaces(self):
    if self._class_sub is not None:
      self.destroy_subscription(self._class_sub)
      self._class_sub = None

    if self._in_img_sub is not None:
      self.destroy_subscription(self._in_img_sub)
      self._in_img_sub = None

    if self._prompt_srv is not None:
      self.destroy_service(self._prompt_srv)
      self._prompt_srv = None

  def _release_resources(self):
    if self._image_pub is not None:
      self.destroy_publisher(self._image_pub)
      self._image_pub = None

    if self._detections_pub is not None:
      self.destroy_publisher(self._detections_pub)
      self._detections_pub = None

    self._bridge = None
    self._processor = None

    if self._model is not None:
      del self._model
      self._model = None
      if torch.cuda.is_available():
        torch.cuda.empty_cache()
      self.get_logger().info("MODEL deleted")

    if self._tracker is not None:
      self._tracker = None
      self.get_logger().info("TRACKER deleted")

  def _image_callback(self, msg: Image):
    if len(self._classes) > 0:
      try:
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        results = self._request_detections(
          self._model,
          self._processor,
          cv_image,
          self._classes,
          self._task
        )

        detections = results[0]
        boxes, scores, text_labels = detections["boxes"], detections["scores"], detections["text_labels"]

        detections_msg = self._generate_detections_msg(boxes, scores, text_labels, msg.header.frame_id)
        
        if self._tracking:
          detections_msg = self._assign_id_with_tracker(cv_image, detections_msg)

        self._detections_pub.publish(detections_msg)

        if self._debug:
          dbg_img_cv = self._parse_detections_for_debug(detections_msg, cv_image)
          dbg_img = self._bridge.cv2_to_imgmsg(dbg_img_cv)
          self._image_pub.publish(dbg_img)

      except Exception as e:
        self.get_logger().error(f"Error processing image: {e}")
        raise
    else:
      detections_msg = Detection2DArray()
      
      detections_msg.header = Header()
      detections_msg.header.frame_id = ""
      detections_msg.header.stamp = self.get_clock().now().to_msg()

      detections_msg.detections = []

      self._detections_pub.publish(detections_msg)
    

  def _generate_detections_msg(self, boxes, scores, text_labels, frame_id):
    header = Header()
    header.frame_id = frame_id
    header.stamp = self.get_clock().now().to_msg()

    detections_msg = Detection2DArray()
    detections_msg.header = header
    for box, score, text_label in zip(boxes, scores, text_labels):
        box = [round(i, 2) for i in box.tolist()]

        x0, y0, x1, y1 = [int(round(i)) for i in box]
        height = float(y1 - y0)
        width = float(x1 - x0)

        cx = float(x0+x1)/2
        cy = float(y0+y1)/2

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = text_label
        hypothesis.hypothesis.score = round(score.item(), 2)

        detection = Detection2D()
        detection.header = header
        detection.bbox.size_x = width
        detection.bbox.size_y = height
        detection.bbox.center.position.x = cx
        detection.bbox.center.position.y = cy
        detection.results.append(hypothesis)
    
        detections_msg.detections.append(detection)

    return detections_msg

  def _request_detections(self, model, processor, image, text_labels, task_label=None):
      # PREPARE INPUTS
      #self.get_logger().info("Preparando input")

      if task_label is not None:
          inputs = processor(image,
                            text=text_labels,
                            task=task_label,
                            return_tensors="pt")
      else:
          inputs = processor(image,
                            text=text_labels,
                            return_tensors="pt")

      # MOVE INPUTS TO GPU
      #self.get_logger().info("Moviendo input")

      for k, v in inputs.items():
          inputs[k] = v.to(self._model_device)


      # CALL THE MODEL
      #self.get_logger().info("Llamando al modelo")

      with torch.no_grad():
          outputs = model(**inputs)


      # POST PROCESS OUTPUTS (bounding boxes and class logits)
      #self.get_logger().info("Procesando outputs")

      height, width = image.shape[:2]

      results = processor.post_process_grounded_object_detection(
          outputs,
          target_sizes=[(height, width)],
          text_labels=text_labels,
          threshold=POST_PROCESS_THRESHOLD,
          nms_threshold=NMS_THRESHOLD,
      )

      return results
  
  # ---------- TRACKING -----------
  def _read_tracking_parameters(self):
    try:
      tracking_params = {
        'max_age':       self.get_parameter('max_age').get_parameter_value().integer_value,
        'min_hits':      self.get_parameter('min_hits').get_parameter_value().integer_value,
        'iou_threshold': self.get_parameter('iou_threshold').get_parameter_value().double_value,
        'min_conf':      self.get_parameter('min_conf').get_parameter_value().double_value,
        'max_cos_dist':  self.get_parameter('max_cos_dist').get_parameter_value().double_value,
        'max_iou_dist':  self.get_parameter('max_iou_dist').get_parameter_value().double_value,
        'nn_budget':     self.get_parameter('nn_budget').get_parameter_value().integer_value,
        'mc_lambda':     self.get_parameter('mc_lambda').get_parameter_value().double_value,
        'ema_alpha':     self.get_parameter('ema_alpha').get_parameter_value().double_value,
        'reid_weights':  self.get_parameter('reid_weights').get_parameter_value().string_value,
        'device':        self.get_parameter('device').get_parameter_value().string_value,
        'image_topic':   self.get_parameter('image_topic').get_parameter_value().string_value,
      }
    except Exception as e:
      self.get_logger().error(f"Parameter error: {e}")
      return None

    return tracking_params

  def _resolve_device(self, device_str: str) -> torch.device:
    if device_str:
      return torch.device(device_str)
    return torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

  def _resolve_weights(self, weights_str: str) -> Path:
    if weights_str:
      weights_path = Path(weights_str)
      if weights_path.exists():
        self.get_logger().info(f"Found weights file: {weights_path}")
        return weights_path

      self.get_logger().warn(
        f"reid_weights path {weights_path} does not exist - using default"
      )

    return Path('osnet_x0_25_msmt17.pt')
  
  def _assign_id_with_tracker(self, img, detections_msg):
    dets_np = self._detections_to_numpy(detections_msg.detections)

    if len(dets_np) == 0:
      tracks = self._tracker.update(
        np.empty((0, 6), dtype=np.float32), img
      )
    else:
      tracks = self._tracker.update(dets_np, img)

    if tracks is not None and len(tracks) > 0:
      assigned_detection_indexes = set()
      iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
      for track in tracks:
        x1, y1, x2, y2, track_id, _, _, _ = track
        track_box = np.array([x1, y1, x2, y2], dtype=np.float32)
        detection_index = self._find_best_detection_match(
          track_box,
          dets_np,
          assigned_detection_indexes,
          iou_threshold
        )

        if detection_index is not None:
          detections_msg.detections[detection_index].id = str(int(track_id))
          assigned_detection_indexes.add(detection_index)
       
    return detections_msg

  def _find_best_detection_match(self, track_box, dets_np, assigned_detection_indexes, iou_threshold):
    best_iou = 0.0
    best_index = None

    for i, det in enumerate(dets_np):
      if i in assigned_detection_indexes:
        continue

      detection_box = det[:4]
      iou = self._compute_iou(track_box, detection_box)
      if iou > best_iou:
        best_iou = iou
        best_index = i

    if best_iou < iou_threshold:
      return None

    return best_index

  def _compute_iou(self, box_a, box_b):
    x_left = max(box_a[0], box_b[0])
    y_top = max(box_a[1], box_b[1])
    x_right = min(box_a[2], box_b[2])
    y_bottom = min(box_a[3], box_b[3])

    intersection_width = max(0.0, x_right - x_left)
    intersection_height = max(0.0, y_bottom - y_top)
    intersection_area = intersection_width * intersection_height

    box_a_area = max(0.0, box_a[2] - box_a[0]) * max(0.0, box_a[3] - box_a[1])
    box_b_area = max(0.0, box_b[2] - box_b[0]) * max(0.0, box_b[3] - box_b[1])
    union_area = box_a_area + box_b_area - intersection_area

    if union_area <= 0.0:
      return 0.0

    return intersection_area / union_area

  def _detections_to_numpy(self, detections) -> np.ndarray:
    """Convert Detection2D[] to boxmot input: float32 array (N, 6)."""
    rows = []
    for det in detections:
      if len(det.results) == 0:
        continue

      cx = det.bbox.center.position.x
      cy = det.bbox.center.position.y
      w  = det.bbox.size_x
      h  = det.bbox.size_y

      x1 = cx - w / 2.0
      y1 = cy - h / 2.0
      x2 = cx + w / 2.0
      y2 = cy + h / 2.0

      rows.append([x1, y1, x2, y2, det.results[0].hypothesis.score, 0.0])

    return np.array(rows, dtype=np.float32) if rows else np.empty((0, 6), dtype=np.float32)

  # ---------- DEBUG --------------
  
  def _parse_detections_for_debug(self, detections_msg, image):
      for det in detections_msg.detections:
        if len(det.results) == 0:
          continue

        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        w = det.bbox.size_x
        h = det.bbox.size_y

        x0, y0 = int(cx - w/2), int(cy - h/2)
        x1, y1 = int(cx + w/2), int(cy + h/2)

        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 0, 255), 2)
        class_id = det.results[0].hypothesis.class_id
        score = det.results[0].hypothesis.score
        label = f"{class_id} ({det.id}) {score:.2f}"
        cv2.putText(image, label, (x0, y0 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

      image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

      # Adapt image to good size
      height, width = image.shape[:2]

      if (height > MAX_OUTPUT_HEIGHT or width > MAX_OUTPUT_WIDTH):
          out_img = cv2.resize(image_bgr, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
      else:
          out_img = image_bgr

      return out_img

# -----------------------------------------------------------------------------------


def main(args=None):
  rclpy.init(args=args)

  node = OmdetNode()

  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
