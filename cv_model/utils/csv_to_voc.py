import os
import shutil
import pandas as pd
from lxml import etree
from PIL import Image
import random

def create_voc_xml(row, img_size, output_dir):
    annotation = etree.Element("annotation")

    # Basic image info
    filename = etree.SubElement(annotation, "filename")
    filename.text = row["filename"]
    
    folder = etree.SubElement(annotation, "folder")
    folder.text = "traffic_dataset"
    
    # Source information
    source = etree.SubElement(annotation, "source")
    database = etree.SubElement(source, "database")
    database.text = "traffic_dataset"
    annotation_source = etree.SubElement(source, "annotation")
    annotation_source.text = "custom"
    image_source = etree.SubElement(source, "image")
    image_source.text = "custom"

    # Size information
    size = etree.SubElement(annotation, "size")
    width = etree.SubElement(size, "width")
    width.text = str(img_size[0])
    height = etree.SubElement(size, "height")
    height.text = str(img_size[1])
    depth = etree.SubElement(size, "depth")
    depth.text = str(3)  # Assuming RGB images

    # Segmented
    segmented = etree.SubElement(annotation, "segmented")
    segmented.text = "0"

    # Object information
    object_elem = etree.SubElement(annotation, "object")
    name = etree.SubElement(object_elem, "name")
    name.text = row["class"]
    
    pose = etree.SubElement(object_elem, "pose")
    pose.text = "unspecified"
    
    truncated = etree.SubElement(object_elem, "truncated")
    truncated.text = "0"
    
    difficult = etree.SubElement(object_elem, "difficult")
    difficult.text = "0"
    
    bndbox = etree.SubElement(object_elem, "bndbox")
    xmin = etree.SubElement(bndbox, "xmin")
    xmin.text = str(int(row["x1"]))
    ymin = etree.SubElement(bndbox, "ymin")
    ymin.text = str(int(row["y1"]))
    xmax = etree.SubElement(bndbox, "xmax")
    xmax.text = str(int(row["x2"]))
    ymax = etree.SubElement(bndbox, "ymax")
    ymax.text = str(int(row["y2"]))

    # Create XML file
    xml_str = etree.tostring(annotation, pretty_print=True, encoding='utf-8').decode('utf-8')
    
    # Save XML file
    base_filename = os.path.splitext(row["filename"])[0]
    with open(os.path.join(output_dir, f"{base_filename}.xml"), "w") as f:
        f.write(xml_str)

def create_splits(image_list, output_dir, train_ratio=0.8, val_ratio=0.1, test_ratio=0.1):
    # Shuffle the images
    random.shuffle(image_list)
    
    # Calculate split sizes
    total = len(image_list)
    train_size = int(total * train_ratio)
    val_size = int(total * val_ratio)
    
    # Split the images
    train_images = image_list[:train_size]
    val_images = image_list[train_size:train_size + val_size]
    test_images = image_list[train_size + val_size:]
    
    # Create trainval list
    trainval_images = train_images + val_images
    
    # Write the splits
    splits = {
        'train.txt': train_images,
        'val.txt': val_images,
        'test.txt': test_images,
        'trainval.txt': trainval_images
    }
    
    for filename, images in splits.items():
        with open(os.path.join(output_dir, filename), 'w') as f:
            f.write('\n'.join(images))
    
    # Print statistics
    print(f"\nDataset split complete:")
    print(f"Total images: {total}")
    print(f"Train: {len(train_images)} images ({len(train_images)/total*100:.1f}%)")
    print(f"Val: {len(val_images)} images ({len(val_images)/total*100:.1f}%)")
    print(f"Test: {len(test_images)} images ({len(test_images)/total*100:.1f}%)")
    print(f"Trainval: {len(trainval_images)} images ({len(trainval_images)/total*100:.1f}%)")

def csv_to_voc(csv_file, images_dir, output_base_dir):
    # Create output directories
    output_img_dir = os.path.join(output_base_dir, "images")
    output_voc_dir = os.path.join(output_base_dir, "voc")
    os.makedirs(output_img_dir, exist_ok=True)
    os.makedirs(output_voc_dir, exist_ok=True)

    # Read CSV file
    df = pd.read_csv(csv_file)
    
    # Filter for desired classes
    desired_classes = ['stop', 'speedLimit25']
    df = df[df['class'].isin(desired_classes)]
    
    # Keep track of processed images
    processed_images = set()
    image_list = []

    # Process each row
    for _, row in df.iterrows():
        img_path = os.path.join(images_dir, row["filename"])
        if not os.path.exists(img_path):
            print(f"Warning: Image {img_path} not found, skipping...")
            continue

        # Get image size
        with Image.open(img_path) as img:
            img_size = img.size + (3,)  # width, height, channels
        
        # Create VOC XML
        create_voc_xml(row, img_size, output_voc_dir)
        
        # Copy image if not already copied
        if row["filename"] not in processed_images:
            shutil.copy2(img_path, os.path.join(output_img_dir, row["filename"]))
            processed_images.add(row["filename"])
            
            # Add to image list (without extension)
            base_filename = os.path.splitext(row["filename"])[0]
            image_list.append(base_filename)
    
    # Save image list
    with open(os.path.join(output_base_dir, "image_list.txt"), "w") as f:
        f.write("\n".join(sorted(image_list)))
    
    print(f"Processed {len(processed_images)} images")
    print(f"Images saved to: {output_img_dir}")
    print(f"Annotations saved to: {output_voc_dir}")
    print(f"Image list saved to: {os.path.join(output_base_dir, 'image_list.txt')}")
    
    # Create dataset splits
    create_splits(sorted(image_list), output_base_dir)

if __name__ == "__main__":
    # Get the absolute path to the script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.dirname(script_dir)  # cv_model directory
    
    # Paths relative to cv_model directory
    csv_file = os.path.join(base_dir, "data", "tiny_lisa", "annotations.csv")
    images_dir = os.path.join(base_dir, "data", "tiny_lisa", "images")
    output_dir = os.path.join(base_dir, "data", "converted")
    
    csv_to_voc(csv_file, images_dir, output_dir)
