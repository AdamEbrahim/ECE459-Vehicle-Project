from ultralytics import YOLO
import cv2
import os
import time

# Load your YOLOv8 model
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, 'models', 'best.pt')
model = YOLO(model_path)

# Open webcam
cap = cv2.VideoCapture(0)  # Use 0 for built-in webcam, 1 for external camera

# Adjust these parameters
CONFIDENCE_THRESHOLD = 0.5  # Minimum confidence to display detection
FRAME_DELAY = 0.01  # Time delay (in seconds) between frames

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Perform inference
    results = model(frame, conf=CONFIDENCE_THRESHOLD)

    # Filter and display detections
    detection_info = []  # Store detections to print in the console
    for detection in results[0].boxes.data.tolist():
        x1, y1, x2, y2, confidence, class_id = detection
        if confidence >= CONFIDENCE_THRESHOLD:
            detection_info.append({
                "class": model.names[int(class_id)],
                "confidence": round(confidence, 2),
                "bbox": [int(x1), int(y1), int(x2), int(y2)]
            })

    # Print detections for debugging
    for detected_obj in detection_info:
        print(f"Detected: {detected_obj['class']} | Confidence: {detected_obj['confidence']}")

    # Visualize detections on the frame
    annotated_frame = results[0].plot()

    # Display the frame
    cv2.imshow("YOLOv8 Detection", annotated_frame)

    # Add a delay to slow down frame display
    time.sleep(FRAME_DELAY)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
