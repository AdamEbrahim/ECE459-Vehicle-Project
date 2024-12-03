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
        
        # Export to TensorRT format with specific output names
        trt_path = os.path.join(current_dir, 'models', 'best.trt')
        
        # Export with specific settings for TensorRT and jetson.inference
        success = model.export(format='trt', 
                             opset=11,      # TensorRT compatible
                             simplify=True,  # Simplify model graph
                             dynamic=False,  # Fixed batch size for better compatibility
                             imgsz=640,     # Fixed image size
                             output_names=['output0', 'bboxes', 'scores', 'labels'])  # Match expected names
        
        if success:
            print(f"Successfully exported model to {trt_path}")
            print("\nVerifying model structure...")
            
            # Print model verification steps
            print("\nTo verify the model on Jetson:")
            print("1. Check TensorRT model:")
            print("   /usr/src/tensorrt/bin/trtexec --trt=best.trt --verbose")
            print("\n2. Look for these in the output:")
            print("   - Input shape should be: (1, 3, 640, 640)")
            print("   - Should see output layers: output0, bboxes, scores, labels")
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
        print("1. Push to git and pull on Jetson")
        print("2. Run model verification command on Jetson")
        print("3. Update detection node if needed based on verification output")
    else:
        print("\nConversion failed. Please check the error messages above.")
