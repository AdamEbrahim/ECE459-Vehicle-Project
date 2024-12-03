from ultralytics import YOLO
import os
import sys

def convert_yolo_model():
    try:
        # Get absolute path to the model
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'models', 'best.pt')
        
        if not os.path.exists(model_path):
            print(f"Error: Model file not found at {model_path}")
            return False
            
        print(f"Loading model from {model_path}")
        model = YOLO(model_path)
        
        # Export to ONNX format
        onnx_path = os.path.join(current_dir, 'models', 'best.onnx')
        
        # Export with specific settings for TensorRT
        success = model.export(format='onnx', 
                             opset=11,      # TensorRT compatible
                             simplify=True,  # Simplify model graph
                             dynamic=False,  # Fixed batch size for better compatibility
                             imgsz=640)     # Fixed image size
        
        if success:
            print(f"Successfully exported model to {onnx_path}")
            print("You can now copy this model to your Jetson")
            return True
        else:
            print("Export failed")
            return False
            
    except Exception as e:
        print(f"Error during conversion: {str(e)}")
        return False

if __name__ == '__main__':
    if convert_yolo_model():
        print("\nNext steps:")
        print("1. Copy the exported model to your Jetson:")
        print("   scp models/best.onnx ece459@ece459-desktop:~/ros2_ws/install/ros_comm/lib/python3.6/site-packages/ros_comm/computer_vision/models/")
        print("\n2. Update the model path in yolo_detection_optimized.py to use 'best.onnx'")
    else:
        print("\nConversion failed. Please check the error messages above.")
