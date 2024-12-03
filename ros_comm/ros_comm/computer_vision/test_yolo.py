from ultralytics import YOLO
import cv2

# Load the model
model = YOLO('models/best.pt')  # or best.engine if you want to test the TensorRT version

# Open the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    # Run inference
    results = model(frame, verbose=True)
    
    # Draw results on frame
    annotated_frame = results[0].plot()
    
    # Display the frame
    cv2.imshow("YOLOv8 Detection", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
