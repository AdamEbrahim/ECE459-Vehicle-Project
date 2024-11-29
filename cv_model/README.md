# Traffic Sign Detection Model

A YOLOv8-based computer vision model for detecting traffic signs and signals in real-time, designed for autonomous vehicle navigation.

## Project Overview

This project implements a real-time traffic sign detection system using YOLOv8-nano, trained on the Tiny LISA dataset. The model is optimized for deployment on NVIDIA Jetson platforms and integration with ROS2.

### Current Features
- Detection of 6 traffic sign classes:
  * Stop signs
  * Yield signs
  * Signal Ahead warnings
  * Pedestrian Crossing signs
  * Speed Limit 25 signs
  * Speed Limit 35 signs
- Real-time inference capability (~6.3ms per frame)
- High accuracy (93.8% mAP50, 78.4% mAP50-95)

## Directory Structure
```
cv_model/
├── data/                  # Dataset and configuration
│   └── tiny_lisa/        # Processed LISA dataset
├── models/               # Model architecture
│   └── yolov8n.yaml     # YOLOv8-nano configuration
├── runs/                 # Training outputs
│   └── detect/
│       └── train2/      # Latest training run
├── utils/               # Utility scripts
├── test_model.py        # Model testing script
└── train.py            # Training script
```

## Setup and Training

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Dataset preparation:
```bash
python utils/convert_annotations.py  # Convert annotations to YOLO format
python utils/split_dataset.py       # Split into train/val/test sets
```

3. Train the model:
```bash
python train.py
```

## Model Performance

Current model metrics:
- Overall mAP50: 93.8%
- Overall mAP50-95: 78.4%
- Per-class performance:
  * Stop signs: 87.5%
  * Yield signs: 91.7%
  * Signal Ahead: 88.7%
  * Pedestrian Crossing: 97.3%
  * Speed Limit 25: 100%
  * Speed Limit 35: 97.8%

## Future Development Plans

### 1. ROS2 Integration
- Create a ROS2 node for real-time inference
- Implement message publishing for detected signs
- Configure camera feed subscription
- Add visualization tools for debugging

### 2. Jetson Deployment
- Optimize model for Jetson hardware
- Implement TensorRT acceleration
- Profile performance on Jetson
- Set up automatic model loading on boot

### 3. Dataset Enhancement
To improve real-world performance, the following data collection steps are planned:

1. Additional Sign Classes:
   - Traffic lights (Red, Yellow, Green)
   - Additional speed limits

2. Real Conditions Data:
   - Collect images from robot's actual camera
   - Capture signs at various:
     * Times of day
     * Weather conditions
     * Angles and distances
     * Lighting conditions

3. Data Collection Process:
   ```bash
   # 1. Collect images from robot
   ros2 bag record /camera/image_raw

   # 2. Label new images
   # Use labelImg or similar tool
   
   # 3. Convert and add to dataset
   python utils/convert_annotations.py --new_data path/to/new/data
   ```

4. Fine-tuning Process:
   - Start with current weights
   - Train on combined dataset
   - Validate in real conditions
   - Iterate based on performance

## Testing

To test the model on sample images:
```bash
python test_model.py
```

Results will be saved in the `test_results/` directory.
This project is licensed under the MIT License - see the LICENSE file for details.

