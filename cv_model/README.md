# Traffic Sign Detection Model

### **Steps to Train the Adjusted Model on New Data**

#### **Step 1: Collect Custom Dataset**
The key to training a custom model is collecting a dataset that resembles the environment your bot will operate in.

1. **Attach Your Camera to the Jetson Nano**:
   - Set up your miniature traffic signs and objects in the bot’s track environment.
   - Use the camera feed to capture images.

2. **Use a Dataset Collection Script**:
   - Modify or use `basic_cv_detection.py` to capture and save images from the camera.
   - Save captured images in a folder like `data/images/`.

3. **Annotate Images**:
   - Use an annotation tool like [LabelImg](https://github.com/heartexlabs/labelImg) to label the traffic signs and objects.
   - Save annotations in Pascal VOC format (XML files).

4. **Organize Dataset**:
   - Structure your dataset as follows:
     ```plaintext
     data/
     ├── images/
     │   ├── train/
     │   ├── val/
     │   └── test/
     ├── annotations/
     │   ├── train/
     │   ├── val/
     │   └── test/
     ```

---

#### **Step 2: Fine-Tune SSD-MobileNet**
1. **Set Up Training Environment**:
   - Ensure PyTorch is installed on your Jetson Nano:
     ```bash
     pip3 install torch torchvision
     ```
   - Install the Jetson-Inference training tools:
     ```bash
     cd jetson-inference/python/training/detection/ssd/
     ```

2. **Train the Model**:
   - Use the fine-tuning script to train on your dataset:
     ```bash
     python3 train_ssd.py --data=data/ --model-dir=ros_comm/computer_vision/models
     ```

3. **Training Details**:
   - `data/`: Path to your dataset directory.
   - `ros_comm/computer_vision/models`: Directory where the trained model will be saved.
   - During training:
     - Logs will show training progress.
     - The script automatically adjusts the pre-trained SSD-MobileNet for your custom classes.

---

#### **Step 3: Convert Model to TensorRT**
1. **Export Model to ONNX**:
   ```bash
   python3 onnx_export.py --model-dir=ros_comm/computer_vision/models --output=ros_comm/computer_vision/models/ssd-mobilenet-v2.onnx
   ```

2. **Optimize with TensorRT**:
   - Convert the ONNX model to TensorRT:
     ```bash
     /usr/src/tensorrt/bin/trtexec --onnx=ros_comm/computer_vision/models/ssd-mobilenet-v2.onnx --saveEngine=ros_comm/computer_vision/models/ssd-mobilenet-v2.engine
     ```

