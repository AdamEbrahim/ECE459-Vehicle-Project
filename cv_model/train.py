from ultralytics import YOLO
import os
import multiprocessing

def main():
    # Get the absolute paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_yaml = os.path.join(current_dir, "data", "tiny_lisa.yaml")
    model_yaml = os.path.join(current_dir, "models", "yolov8n.yaml")

    # Initialize model from local YAML configuration
    model = YOLO(model_yaml)

    # Train the model with optimized parameters
    model.train(
        data=data_yaml,
        epochs=150,
        imgsz=640,
        batch=16,
        patience=50,
        verbose=True,
        cache=True  # cache images for faster training
    )

if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
