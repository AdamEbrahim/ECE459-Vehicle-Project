import os
import random
import shutil

def split_dataset(image_dir, label_dir, output_base, train_ratio=0.7, val_ratio=0.15, test_ratio=0.15):
    """Split dataset into train, validation and test sets."""
    # Create output directories
    splits = ['train', 'val', 'test']
    for split in splits:
        os.makedirs(os.path.join(output_base, split, 'images'), exist_ok=True)
        os.makedirs(os.path.join(output_base, split, 'labels'), exist_ok=True)

    # Get list of all files (without extensions)
    files = [os.path.splitext(f)[0] for f in os.listdir(image_dir) if f.endswith('.png')]
    
    # Shuffle files
    random.shuffle(files)
    
    # Calculate split sizes
    n_files = len(files)
    n_train = int(n_files * train_ratio)
    n_val = int(n_files * val_ratio)
    
    # Split files
    train_files = files[:n_train]
    val_files = files[n_train:n_train + n_val]
    test_files = files[n_train + n_val:]
    
    # Copy files to respective directories
    splits_files = {
        'train': train_files,
        'val': val_files,
        'test': test_files
    }
    
    for split, file_list in splits_files.items():
        for file in file_list:
            # Copy image
            src_img = os.path.join(image_dir, f"{file}.png")
            dst_img = os.path.join(output_base, split, 'images', f"{file}.png")
            shutil.copy2(src_img, dst_img)
            
            # Copy label if exists
            src_label = os.path.join(label_dir, f"{file}.txt")
            if os.path.exists(src_label):
                dst_label = os.path.join(output_base, split, 'labels', f"{file}.txt")
                shutil.copy2(src_label, dst_label)
    
    # Print statistics
    print(f"Dataset split complete:")
    print(f"Train set: {len(train_files)} images")
    print(f"Validation set: {len(val_files)} images")
    print(f"Test set: {len(test_files)} images")

if __name__ == "__main__":
    # Paths
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tiny_lisa_dir = os.path.join(base_dir, "data", "tiny_lisa")
    
    # Source directories (from tiny_lisa)
    image_dir = os.path.join(tiny_lisa_dir, 'images')
    label_dir = os.path.join(tiny_lisa_dir, 'labels')
    
    # Output directory for splits (same as tiny_lisa)
    output_base = tiny_lisa_dir
    
    # Split dataset
    split_dataset(image_dir, label_dir, output_base)
