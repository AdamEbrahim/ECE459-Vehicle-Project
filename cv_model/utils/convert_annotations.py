import os
import pandas as pd
from PIL import Image
import shutil

# Define class mapping for the classes we want
class_map = {
    "stop": 0,
    "yield": 1,
    "signalAhead": 2,
    "pedestrianCrossing": 3,
    "speedLimit25": 4,
    "speedLimit35": 5
}

def convert_annotations(csv_file, image_dir, label_dir, filtered_image_dir):
    """Convert CSV annotations to YOLO format and filter unwanted classes."""
    os.makedirs(label_dir, exist_ok=True)
    # No need to create filtered_image_dir since we're using the same directory

    # Read CSV
    df = pd.read_csv(csv_file)
    
    # Filter for classes we want
    df = df[df['class'].isin(class_map.keys())]
    
    # Process each annotation
    processed_files = set()
    for _, row in df.iterrows():
        img_path = os.path.join(image_dir, row['filename'])
        if not os.path.exists(img_path):
            print(f"Image not found: {img_path}")
            continue

        # Get image dimensions
        with Image.open(img_path) as img:
            img_width, img_height = img.size

        # Calculate YOLO values
        x_center = ((row['x1'] + row['x2']) / 2) / img_width
        y_center = ((row['y1'] + row['y2']) / 2) / img_height
        width = (row['x2'] - row['x1']) / img_width
        height = (row['y2'] - row['y1']) / img_height
        class_id = class_map[row['class']]

        # Write to YOLO label file
        label_file = os.path.join(label_dir, os.path.splitext(row['filename'])[0] + '.txt')
        
        # Track processed files (no need to copy since we're using same directory)
        if row['filename'] not in processed_files:
            processed_files.add(row['filename'])
        
        # Write to label file (using write mode to avoid duplicates)
        with open(label_file, 'w' if row['filename'] not in processed_files else 'a') as f:
            f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

    print(f"Processed {len(processed_files)} images with {len(df)} annotations")
    return processed_files

if __name__ == "__main__":
    # Paths
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tiny_lisa_dir = os.path.join(base_dir, "data", "tiny_lisa")
    csv_file = os.path.join(tiny_lisa_dir, "annotations.csv")
    image_dir = os.path.join(tiny_lisa_dir, "images")  # Original images
    
    # Output directories (now directly in tiny_lisa)
    filtered_image_dir = os.path.join(tiny_lisa_dir, "images")  # Keep filtered images in same directory
    label_dir = os.path.join(tiny_lisa_dir, "labels")
    
    # Convert annotations
    processed_files = convert_annotations(csv_file, image_dir, label_dir, filtered_image_dir)
    print("Conversion completed successfully!")
