"""
YOLO to TensorFlow Lite Conversion Script

This script converts YOLO models (YOLOv5, YOLOv8, etc.) to TensorFlow Lite format
for deployment on mobile devices, edge devices, and embedded systems.
"""

import argparse
from ultralytics import YOLO

def convert_yolo_to_tflite(model_path, output_name=None, img_size=640, int8=False, data_yaml=None):
    """
    Convert YOLO model to TensorFlow Lite format
    
    Args:
        model_path: Path to YOLO .pt model file
        output_name: Output filename (without extension)
        img_size: Input image size for the model
        int8: Whether to use INT8 quantization
        data_yaml: Path to dataset YAML file (required for INT8 quantization)
    """
    print(f"Loading YOLO model from: {model_path}")
    model = YOLO(model_path)
    
    # Determine output name
    if output_name is None:
        output_name = model_path.replace('.pt', '')
    
    print(f"\nConverting to TensorFlow Lite...")
    print(f"Image size: {img_size}")
    print(f"INT8 quantization: {int8}")
    
    # Convert to TFLite
    # The export method handles the conversion
    if int8:
        if data_yaml is None:
            raise ValueError("data_yaml must be provided for INT8 quantization")
        model.export(
            format='tflite',
            imgsz=img_size,
            int8=True,
            data=data_yaml
        )
    else:
        model.export(
            format='tflite',
            imgsz=img_size
        )
    
    print(f"\n✓ Conversion complete!")
    print(f"TFLite model saved as: {output_name}_saved_model/{output_name}_float16.tflite")
    if int8:
        print(f"INT8 model saved as: {output_name}_saved_model/{output_name}_int8.tflite")

def main():
    parser = argparse.ArgumentParser(description='Convert YOLO model to TensorFlow Lite')
    parser.add_argument('--model', type=str, required=True,
                        help='Path to YOLO .pt model file')
    parser.add_argument('--output', type=str, default=None,
                        help='Output filename (without extension)')
    parser.add_argument('--img-size', type=int, default=640,
                        help='Input image size (default: 640)')
    parser.add_argument('--int8', action='store_true',
                        help='Enable INT8 quantization for smaller model size')
    parser.add_argument('--data', type=str, default=None,
                        help='Path to dataset YAML file (required for INT8)')
    
    args = parser.parse_args()
    
    convert_yolo_to_tflite(
        model_path=args.model,
        output_name=args.output,
        img_size=args.img_size,
        int8=args.int8,
        data_yaml=args.data
    )

if __name__ == '__main__':
    main()