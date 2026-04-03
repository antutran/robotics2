import cv2
import numpy as np
import serial
import time
from ultralytics import YOLO
from lane_processor import LaneProcessor


SERIAL_PORT = '/dev/cu.usbmodem00000000001A1' 
BAUD_RATE = 115200
MODEL_PATH = "best.pt"
VIDEO_SOURCE = 0 # Webcam

# Lane HSV range
LOWER_YELLOW = np.array([10, 30, 60])
UPPER_YELLOW = np.array([40, 255, 255])

def main():
    # 1. Initialize YOLO
    print(f"🔄 Loading YOLO Model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    
    # 2. Initialize Camera
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # 3. Initialize Lane Processor
    lane_proc = LaneProcessor(width, height, roi_factor=0.6)
    
    # 4. Initialize Serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        print(f"✅ Serial connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"❌ Serial Error: {e}")
        ser = None

    print("\n🚀 Pilot System Ready. Waiting for STM32 Auto Mode...")

    prev_time = 0
    auto_mode = False
    run_enabled = False
    ai_enabled = False
    lane_enabled = False

    def safe_send(msg):
        nonlocal ser
        if ser:
            try:
                ser.write(msg.encode())
            except (serial.SerialException, OSError):
                print("⚠️ Serial disconnected. Reconnecting...")
                ser = None
        else:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
                print(f"✅ Serial reconnected.")
            except:
                pass

    while True:
        # A. Read STM32 Status
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("S:"):
                    # Format S:mode,run,ai,lane
                    parts = line[2:].split(',')
                    if len(parts) == 4:
                        auto_mode = (int(parts[0]) == 0) # MOVE_AUTO is 0
                        run_enabled = (int(parts[1]) == 1)
                        ai_enabled = (int(parts[2]) == 1)
                        lane_enabled = (int(parts[3]) == 1)
            except:
                pass

        # B. Image Processing
        ret, frame = cap.read()
        if not ret: break
        
        display_frame = frame.copy()
        
        if auto_mode:
            # --- 1. Lane Processing ---
            roi = lane_proc.get_roi(frame)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
            
            left, right, center = lane_proc.process_mask(mask)
            heading, lateral = lane_proc.calculate_errors(center)
            
            # --- 2. AI Processing ---
            best_det = -1 # None
            if ai_enabled:
                # Define priority: Obstacle (8) > Red (5) > Slow (3) > Yellow (6) > others
                # Indices based on previous mapping: {0:oneway, 1:park, 2:turn_right, 3:slow, 4:green, 5:red, 6:yellow, 7:crosswalk, 8:obstacle}
                priority_map = {8: 100, 5: 90, 3: 80, 6: 70, 2: 60, 7: 50} 
                current_top_priority = -1
                
                results = model.predict(frame, stream=True, conf=0.5, verbose=False)
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cls_idx = int(box.cls[0])
                        conf = float(box.conf[0])
                        label = r.names[cls_idx]
                        
                        # Get priority of this detection (default to 0 if not special)
                        prio = priority_map.get(cls_idx, 10)
                        
                        if prio > current_top_priority:
                            current_top_priority = prio
                            best_det = cls_idx
                        
                        # Draw UI
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(display_frame, f"{label} {conf:.2f}", (x1, y1-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 3. Communications (Safe Write)
            safe_send(f"D:{best_det + 1}\n") # If no det, sends D:0
            if run_enabled:
                safe_send(f"L:{lateral:.2f},H:{heading:.2f}\n")

            # Draw Lane Overlay
            display_frame = lane_proc.draw_overlay(display_frame, left, right, center, heading, lateral)
        else:
            # Manual Mode Overlay
            overlay = display_frame.copy()
            cv2.rectangle(overlay, (0,0), (width, height), (0,0,0), -1)
            cv2.addWeighted(overlay, 0.5, display_frame, 0.5, 0, display_frame)
            cv2.putText(display_frame, "MANUAL MODE - IDLE", (width//4, height//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # C. Performance & Display
        fps = 1 / (time.time() - prev_time) if prev_time != 0 else 0
        prev_time = time.time()
        cv2.putText(display_frame, f"FPS: {fps:.1f}", (width - 120, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Robot Pilot System", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    if ser: ser.close()

if __name__ == "__main__":
    main()
