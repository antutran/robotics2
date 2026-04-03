import cv2
import time
from ultralytics import YOLO

# ===== CONFIGURATION =====
# Path to your YOLO model
MODEL_PATH = "best.pt"

# Webcam settings
CAM_WIDTH = 640
CAM_HEIGHT = 480

# Class names in your model (for reference)
class_names = [
    "turn_right", "crosswalk", "park", "slow_down", 
    "yellow_light", "red_light", "green_light", 
    "one_way", "obstacle"
]

def main():
    # 1. Load the YOLO model
    print(f"🔄 Loading model from {MODEL_PATH}...")
    try:
        model = YOLO(MODEL_PATH)
        print("✅ Model loaded successfully.")
    except Exception as e:
        print(f"❌ Error loading model: {e}")
        return

    # 2. Initialize the webcam
    # Note: No CAP_DSHOW for macOS
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    if not cap.isOpened():
        print("❌ Error: Could not open webcam.")
        return

    print("🚀 YOLO Realtime Test Started. Press 'q' to quit.")

    # Variables for FPS calculation
    prev_time = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break

            # 3. Run Inference
            # stream=True is more memory efficient for video
            results = model.predict(frame, stream=True, conf=0.5, verbose=False)

            detected_classes = []

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Get coordinates
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Get confidence and class index
                    conf = float(box.conf[0])
                    cls_idx = int(box.cls[0])
                    label = r.names[cls_idx] if r.names else str(cls_idx)
                    
                    detected_classes.append(f"{label} ({conf:.2f})")

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw label background
                    label_str = f"{label} {conf:.2%}"
                    (text_w, text_h), baseline = cv2.getTextSize(label_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(frame, (x1, y1 - text_h - 10), (x1 + text_w, y1), (0, 255, 0), -1)

                    # Draw label text
                    cv2.putText(frame, label_str, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

            # 4. Console Logging
            if detected_classes:
                print(f"Detected: {', '.join(detected_classes)}")
            else:
                print("NO OBJECT")

            # 5. FPS Calculation & Display
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
            prev_time = curr_time
            
            cv2.putText(frame, f"FPS: {fps:.1f}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 6. Show the output
            cv2.imshow("YOLO Detection Test", frame)

            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        print("👋 Program closed.")

if __name__ == "__main__":
    main()
