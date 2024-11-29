from ultralytics import YOLO
import cv2
import os
import random

def main():
    # Load the trained model
    model_path = os.path.join('runs', 'detect', 'train2', 'weights', 'best.pt')
    model = YOLO(model_path)
    
    # Get test images path
    test_images_path = os.path.join('data', 'tiny_lisa', 'test', 'images')
    
    # Get list of test images
    test_images = [f for f in os.listdir(test_images_path) if f.endswith(('.jpg', '.png'))]
    
    # Randomly select 5 images
    test_samples = random.sample(test_images, min(5, len(test_images)))
    
    print(f"\nTesting model on {len(test_samples)} random images...")
    
    # Create output directory if it doesn't exist
    output_dir = 'test_results'
    os.makedirs(output_dir, exist_ok=True)
    
    # Run inference on each image
    for img_name in test_samples:
        img_path = os.path.join(test_images_path, img_name)
        
        # Run inference
        results = model(img_path, conf=0.25)  # lower confidence threshold for visualization
        
        # Get the result image with boxes drawn
        res = results[0]
        
        # Save the result
        output_path = os.path.join(output_dir, f'pred_{img_name}')
        res_plotted = res.plot()
        cv2.imwrite(output_path, res_plotted)
        
        # Print detections
        print(f"\nDetections in {img_name}:")
        for box in res.boxes:
            class_id = box.cls[0].item()
            conf = box.conf[0].item()
            class_name = model.names[int(class_id)]
            print(f"- {class_name}: {conf:.2%} confidence")

    print(f"\nResults saved in {output_dir}/")

if __name__ == '__main__':
    main()
