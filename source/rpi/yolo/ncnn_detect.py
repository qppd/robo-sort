import ncnn
import cv2
import numpy as np
import time

class YOLODetector:
    def __init__(self, param_path, bin_path, input_size=640, conf_threshold=0.25, nms_threshold=0.45):
        """
        Initialize YOLO detector with NCNN
        
        Args:
            param_path: Path to .param file
            bin_path: Path to .bin file
            input_size: Model input size (default 640)
            conf_threshold: Confidence threshold for detections
            nms_threshold: NMS IoU threshold
        """
        self.net = ncnn.Net()
        self.net.load_param(param_path)
        self.net.load_model(bin_path)
        
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        # Class names - update these based on your model
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
            'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
            'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
            'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
            'hair drier', 'toothbrush'
        ]
        
        # Colors for visualization
        np.random.seed(42)
        self.colors = np.random.randint(0, 255, size=(len(self.class_names), 3), dtype=np.uint8)
    
    def preprocess(self, image):
        """
        Preprocess image for YOLO inference
        
        Args:
            image: Input BGR image (HxWxC)
            
        Returns:
            mat_in: NCNN Mat object
            scale: Scale factor for coordinate conversion
            pad_w, pad_h: Padding values
        """
        img_h, img_w = image.shape[:2]
        
        # Calculate scale and padding to maintain aspect ratio
        scale = min(self.input_size / img_w, self.input_size / img_h)
        new_w = int(img_w * scale)
        new_h = int(img_h * scale)
        
        # Resize image
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        # Calculate padding
        pad_w = (self.input_size - new_w) // 2
        pad_h = (self.input_size - new_h) // 2
        
        # Add padding (letterbox)
        padded = cv2.copyMakeBorder(
            resized, pad_h, self.input_size - new_h - pad_h, 
            pad_w, self.input_size - new_w - pad_w,
            cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )
        
        # Convert to RGB
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        
        # Create NCNN Mat from numpy array and normalize
        mat_in = ncnn.Mat.from_pixels(rgb, ncnn.Mat.PixelType.PIXEL_RGB, self.input_size, self.input_size)
        
        # Normalize to [0, 1]
        mean_vals = [0.0, 0.0, 0.0]
        norm_vals = [1/255.0, 1/255.0, 1/255.0]
        mat_in.substract_mean_normalize(mean_vals, norm_vals)
        
        return mat_in, scale, pad_w, pad_h
    
    def postprocess(self, output, img_w, img_h, scale, pad_w, pad_h):
        """
        Decode YOLO output and apply NMS
        
        Args:
            output: NCNN Mat output from the network
            img_w, img_h: Original image dimensions
            scale: Scale factor from preprocessing
            pad_w, pad_h: Padding values from preprocessing
            
        Returns:
            boxes: List of bounding boxes [x1, y1, x2, y2]
            scores: List of confidence scores
            class_ids: List of class IDs
        """
        # Convert NCNN Mat to numpy array
        # YOLOv8 output shape: [1, 84, 8400] for COCO (4 bbox + 80 classes)
        # Transpose to [8400, 84]
        output_array = np.array(output).reshape(-1, output.h)
        
        # Extract boxes and scores
        boxes_xywh = output_array[:, :4]  # [x_center, y_center, width, height]
        class_scores = output_array[:, 4:]  # Class scores
        
        # Get max score and class id for each detection
        max_scores = np.max(class_scores, axis=1)
        class_ids = np.argmax(class_scores, axis=1)
        
        # Filter by confidence threshold
        mask = max_scores > self.conf_threshold
        boxes_xywh = boxes_xywh[mask]
        max_scores = max_scores[mask]
        class_ids = class_ids[mask]
        
        if len(boxes_xywh) == 0:
            return [], [], []
        
        # Convert from xywh to xyxy format
        boxes_xyxy = np.zeros_like(boxes_xywh)
        boxes_xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2  # x1
        boxes_xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2  # y1
        boxes_xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2  # x2
        boxes_xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2  # y2
        
        # Scale coordinates back to original image
        boxes_xyxy[:, [0, 2]] = (boxes_xyxy[:, [0, 2]] - pad_w) / scale
        boxes_xyxy[:, [1, 3]] = (boxes_xyxy[:, [1, 3]] - pad_h) / scale
        
        # Clip to image boundaries
        boxes_xyxy[:, [0, 2]] = np.clip(boxes_xyxy[:, [0, 2]], 0, img_w)
        boxes_xyxy[:, [1, 3]] = np.clip(boxes_xyxy[:, [1, 3]], 0, img_h)
        
        # Apply NMS
        indices = self.nms(boxes_xyxy, max_scores)
        
        return boxes_xyxy[indices].tolist(), max_scores[indices].tolist(), class_ids[indices].tolist()
    
    def nms(self, boxes, scores):
        """
        Non-Maximum Suppression
        
        Args:
            boxes: numpy array of shape [N, 4] (x1, y1, x2, y2)
            scores: numpy array of shape [N]
            
        Returns:
            keep: List of indices to keep
        """
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            
            inds = np.where(iou <= self.nms_threshold)[0]
            order = order[inds + 1]
        
        return keep
    
    def detect(self, image):
        """
        Run full detection pipeline
        
        Args:
            image: Input BGR image
            
        Returns:
            boxes: List of bounding boxes
            scores: List of confidence scores
            class_ids: List of class IDs
        """
        img_h, img_w = image.shape[:2]
        
        # Preprocess
        mat_in, scale, pad_w, pad_h = self.preprocess(image)
        
        # Inference
        ex = self.net.create_extractor()
        ex.input("in0", mat_in)
        
        ret, mat_out = ex.extract("out0")
        
        # Postprocess
        boxes, scores, class_ids = self.postprocess(mat_out, img_w, img_h, scale, pad_w, pad_h)
        
        return boxes, scores, class_ids
    
    def draw_detections(self, image, boxes, scores, class_ids):
        """
        Draw bounding boxes and labels on image
        
        Args:
            image: Input BGR image
            boxes: List of bounding boxes
            scores: List of confidence scores
            class_ids: List of class IDs
            
        Returns:
            image: Image with drawn detections
        """
        for box, score, class_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = map(int, box)
            
            # Get color for this class
            color = tuple(map(int, self.colors[class_id]))
            
            # Draw bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{self.class_names[class_id]}: {score:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(image, (x1, y1 - label_h - 10), (x1 + label_w, y1), color, -1)
            cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return image


def main():
    # Initialize detector
    detector = YOLODetector(
        param_path="my_model_ncnn_model/model.ncnn.param",
        bin_path="my_model_ncnn_model/model.ncnn.bin",
        input_size=640,
        conf_threshold=0.25,
        nms_threshold=0.45
    )
    
    # Open camera
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    print("Press ESC to exit")
    
    # FPS calculation
    fps_start_time = time.time()
    fps_counter = 0
    fps = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break
        
        # Run detection
        boxes, scores, class_ids = detector.detect(frame)
        
        # Draw results
        frame = detector.draw_detections(frame, boxes, scores, class_ids)
        
        # Calculate and display FPS
        fps_counter += 1
        if fps_counter >= 10:
            fps_end_time = time.time()
            fps = fps_counter / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_counter = 0
        
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Detections: {len(boxes)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display
        cv2.imshow("NCNN YOLO", frame)
        
        # Exit on ESC
        if cv2.waitKey(1) == 27:
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
