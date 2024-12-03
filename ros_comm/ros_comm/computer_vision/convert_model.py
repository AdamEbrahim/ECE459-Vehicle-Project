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
        
        # Export to ONNX format with settings optimized for TensorRT conversion
        onnx_path = os.path.join(current_dir, 'models', 'best.onnx')
        print("\nConverting to ONNX...")
        
        success = model.export(format='onnx',
                             opset=11,        # TensorRT compatible opset
                             simplify=True,    # Simplify model graph
                             dynamic=False,    # Fixed batch size
                             imgsz=640,        # Fixed image size
                             half=True,        # FP16 for better performance
                             input_names=['images'],
                             output_names=['output0'])
        
        if success:
            print(f"\nSuccessfully exported ONNX model to {onnx_path}")
            print(f"ONNX file size: {os.path.getsize(onnx_path)/1024/1024:.2f} MB")
            
            print("\nNext steps:")
            print("1. Copy the ONNX model to your Jetson")
            print("2. On the Jetson, convert to TensorRT engine:")
            print("   trtexec --onnx=best.onnx --saveEngine=best.engine --fp16 --workspace=4096")
            return True
        else:
            print("ONNX export failed")
            return False
            
    except Exception as e:
        print(f"Error during conversion: {str(e)}")
        return False

if __name__ == '__main__':
    convert_yolo_model()
