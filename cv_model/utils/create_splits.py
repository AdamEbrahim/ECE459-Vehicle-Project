import os
import random

def create_splits(image_list_path, output_dir, train_ratio=0.8, val_ratio=0.1, test_ratio=0.1):
    # Read the image list
    with open(image_list_path, 'r') as f:
        images = f.read().splitlines()
    
    # Shuffle the images
    random.shuffle(images)
    
    # Calculate split sizes
    total = len(images)
    train_size = int(total * train_ratio)
    val_size = int(total * val_ratio)
    
    # Split the images
    train_images = images[:train_size]
    val_images = images[train_size:train_size + val_size]
    test_images = images[train_size + val_size:]
    
    # Create trainval list
    trainval_images = train_images + val_images
    
    # Write the splits
    splits = {
        'train.txt': train_images,
        'val.txt': val_images,
        'test.txt': test_images,
        'trainval.txt': trainval_images
    }
    
    for filename, image_list in splits.items():
        with open(os.path.join(output_dir, filename), 'w') as f:
            f.write('\n'.join(image_list))
    
    # Print statistics
    print(f"Dataset split complete:")
    print(f"Total images: {total}")
    print(f"Train: {len(train_images)} images ({len(train_images)/total*100:.1f}%)")
    print(f"Val: {len(val_images)} images ({len(val_images)/total*100:.1f}%)")
    print(f"Test: {len(test_images)} images ({len(test_images)/total*100:.1f}%)")
    print(f"Trainval: {len(trainval_images)} images ({len(trainval_images)/total*100:.1f}%)")

if __name__ == "__main__":
    # Get the script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.dirname(script_dir)  # cv_model directory
    
    # Paths
    image_list_path = os.path.join(base_dir, "data", "converted", "image_list.txt")
    output_dir = os.path.join(base_dir, "data", "converted")
    
    # Create the splits
    create_splits(image_list_path, output_dir)
