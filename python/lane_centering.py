import cv2
import numpy as np
import serial
import time
from lane_processor import LaneProcessor

# ===== CONFIGURATION =====
# Video source (0 for webcam, or path to file)
VIDEO_PATH = 0 

# Serial Port (Adjust for your system, e.g., 'COM3' on Windows or '/dev/cu.usbmodem...' on Mac)
# Use a generic pattern if not sure, but here we expect the user to provide correct one.
SERIAL_PORT = '/dev/cu.usbmodem00000000001A1' 
BAUD_RATE = 115200

# ROI (from video_mask_yellow.py)
ROI_FACTOR = 0.55

# HSV range for yellow - OPTIMIZED
LOWER_YELLOW = np.array([10, 30, 60])
UPPER_YELLOW = np.array([40, 255, 255])

# Create VideoCapture with DSHOW for stability on Windows
cap = cv2.VideoCapture(VIDEO_PATH, cv2.CAP_DSHOW)

if not cap.isOpened():
    # Fallback to default if DSHOW fails
    cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    print(f"❌ Error: Could not open camera {VIDEO_PATH}")
    exit()

# Get properties
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps_val = int(cap.get(cv2.CAP_PROP_FPS)) or 30

# Initialize Serial Port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"✅ Serial connected to {SERIAL_PORT}")
except Exception as e:
    print(f"⚠️ Warning: Could not open serial port {SERIAL_PORT}: {e}")
    ser = None

# Initialize Lane Processor
lane_proc = LaneProcessor(width, height, roi_factor=ROI_FACTOR)

print(f"🚀 Lane Centering Started: {width}x{height} @ {fps_val} FPS")

while True:
    start_time = time.time()
    ret, frame = cap.read()
    if not ret:
        print("End of video stream or error.")
        break

    # 1. Take ROI
    roi = lane_proc.get_roi(frame)
    
    # 2. Convert to HSV & Create mask (Logic from video_mask_yellow.py)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    # [IMPROVED] Use LaneProcessor's morphology instead of manual kernels
    # This includes vertical-emphasis filtering to suppress crosswalk bars
    mask = lane_proc.apply_morphology(mask)

    # 3. Process Lane Geometry
    try:
        left, right, center = lane_proc.process_mask(mask)
        heading, lateral = lane_proc.calculate_errors(center)
        
        # 3b. Send data to STM32 via Serial
        if ser and ser.is_open:
            # Format: L:lateral,H:heading
            # We add \n for the STM32 to potentially use with gets/scanf or similar
            data_to_send = f"L:{lateral:.2f},H:{heading:.2f}\n"
            ser.write(data_to_send.encode('utf-8'))
            
    except Exception as e:
        print(f"Error processing frame: {e}")
        continue

    # 4. Visualization & Overlay
    overlay_frame = lane_proc.draw_overlay(frame, left, right, center, heading, lateral)

    # 5. Performance Calculation
    fps = 1.0 / (time.time() - start_time)
    cv2.putText(overlay_frame, f"FPS: {fps:.1f}", (width - 150, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # 5b. Serial Status Calculation
    ser_status = "CONNECTED" if (ser and ser.is_open) else "DISCONNECTED"
    ser_color = (0, 255, 0) if (ser and ser.is_open) else (0, 0, 255)
    cv2.putText(overlay_frame, f"SERIAL: {ser_status}", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, ser_color, 2)
    cv2.putText(overlay_frame, f"LAT: {lateral:.2f} HEAD: {heading:.2f}", (20, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    # 5b. Log Telemetry to file for checking
    try:
        with open("telemetry.txt", "a") as f:
            f.write(f"{time.time()}, {heading:.2f}, {lateral:.2f}, {1 if left else 0}, {1 if right else 0}\n")
    except Exception as e:
        # Don't crash if logging fails, just print once or ignore
        pass

    # 6. Show Windows (Debug Mode)
    cv2.imshow("Lane Centering Output", overlay_frame)
    cv2.imshow("Yellow Mask ROI", mask)

    # Exit on 'q' or ESC
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("👋 Program Closed.")
