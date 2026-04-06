import cv2
import numpy as np
import serial
import time
import pygame  # Added for better keyboard control
from ultralytics import YOLO
from lane_processor import LaneProcessor

import pygame  # Added for better keyboard control

SERIAL_PORT = '/dev/cu.usbmodem00000000001A1' 
BAUD_RATE = 115200
MODEL_PATH = "best.pt"
VIDEO_SOURCE = 0 # Webcam
pygame.init()
pygame.display.set_mode((100, 100)) # Small window for key focus
pygame.display.set_caption("ROBOT_CMD")

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
                ser.flush() # Ensure it's sent immediately
            except:
                ser = None
        else:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0) # Non-blocking
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
            # --- 1. Boundary Detection (yellow color scan) ---
            roi = lane_proc.get_roi(frame)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
            mask = lane_proc.apply_morphology(mask)
            boundary_detected, fill_ratio = lane_proc.detect_boundary(mask)
            
            # --- 2. AI Processing ---
            best_det = -1 # None
            if ai_enabled:
                priority_map = {8: 100, 5: 90, 1: 85, 3: 80, 0: 75, 6: 70, 2: 60, 7: 50, 9: 45} 
                current_top_priority = -1
                
                green_detected = False
                cross_detected = False
                
                results = model.predict(frame, stream=True, conf=0.08, verbose=False) # Maximum sensitivity
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cls_idx = int(box.cls[0])
                        conf = float(box.conf[0])
                        label = r.names[cls_idx]
                        
                        # --- Per-class Confidence Thresholds (Fine-Tuned Sensitivity) ---
                        if cls_idx in [4, 6]:   thresh = 0.10  # Green, Yellow (Very sensitive)
                        elif cls_idx == 5:    thresh = 0.50  # Red (Sensitive)
                        elif cls_idx == 7:    thresh = 0.20  # Crosswalk (Reduced)
                        elif cls_idx == 9:    thresh = 0.90  # T_mark (Increased to be less sensitive)
                        else:                 thresh = 0.50  # Others
                        
                        if conf < thresh:
                            continue # Skip detections that don't meet their specific threshold
                            
                        if cls_idx == 4: green_detected = True
                        if cls_idx == 7: cross_detected = True

                        # Get priority of this detection
                        prio = priority_map.get(cls_idx, 10)
                        
                        if prio > current_top_priority:
                            current_top_priority = prio
                            best_det = cls_idx
                        
                        # Draw UI
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(display_frame, f"{label} {conf:.2f}", (x1, y1-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # --- New Interaction Logic ---
                # 1. Turn Right Only on Crosswalk + Green Light
                if green_detected and cross_detected:
                    best_det = 2 # Trigger DET_TURN_RIGHT logic on STM32
                    current_top_priority = 101 # Ensure this wins
                
                # 2. Ngã 3, 4 logic removed (now handled as normal detections -> DRIVE straight)
            
            # 3. Communications (Safe Write)
            safe_send(f"D:{best_det + 1}\n") # If no det, sends D:0
            if run_enabled:
                safe_send(f"B:{1 if boundary_detected else 0}\n")

            # Draw Boundary Overlay
            display_frame = lane_proc.draw_overlay(display_frame, boundary_detected, fill_ratio)
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
        
        # Keyboard remote control logic using PYGAME for reliable hold/release
        if not auto_mode:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w or event.key == pygame.K_UP:
                        safe_send("K:U\n")
                    elif event.key == pygame.K_s or event.key == pygame.K_DOWN:
                        safe_send("K:D\n")
                    elif event.key == pygame.K_a or event.key == pygame.K_LEFT:
                        safe_send("K:L\n")
                    elif event.key == pygame.K_d or event.key == pygame.K_RIGHT:
                        safe_send("K:R\n")
                    elif event.key == pygame.K_SPACE:
                        safe_send("K:S\n")
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_w or event.key == pygame.K_UP:
                        safe_send("K:u\n") # Release Up
                    elif event.key == pygame.K_s or event.key == pygame.K_DOWN:
                        safe_send("K:d\n") # Release Down
                    elif event.key == pygame.K_a or event.key == pygame.K_LEFT:
                        safe_send("K:l\n") # Release Left
                    elif event.key == pygame.K_d or event.key == pygame.K_RIGHT:
                        safe_send("K:r\n") # Release Right

    cap.release()
    cv2.destroyAllWindows()
    pygame.quit()
    if ser: ser.close()

if __name__ == "__main__":
    main()
