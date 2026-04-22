import cv2
import time
import os
from ultralytics import YOLO

# ===== CẤU HÌNH =====
# Tự động tìm đường dẫn tới file best.pt ở thư mục gốc
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "best.pt")

# Cài đặt camera
CAM_WIDTH = 640
CAM_HEIGHT = 480

def main():
    # 1. Load model YOLO
    print(f"🔄 Đang tải model từ: {MODEL_PATH}...")
    if not os.path.exists(MODEL_PATH):
        print(f"❌ Lỗi: Không tìm thấy file {MODEL_PATH}")
        return

    try:
        model = YOLO(MODEL_PATH)
        print("✅ Model đã sẵn sàng.")
    except Exception as e:
        print(f"❌ Lỗi khi tải model: {e}")
        return

    # 2. Khởi tạo Camera
    # Trên macOS, index 0 thường là FaceTime HD Camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    if not cap.isOpened():
        print("❌ Lỗi: Không thể mở camera. Hãy kiểm tra quyền truy cập camera trong System Settings.")
        return

    print("🚀 Đang mở luồng camera. Nhấn 'q' để thoát.")

    prev_time = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Không nhận được dữ liệu từ camera.")
                break

            # 3. Chạy Inference (Dự đoán)
            results = model.predict(frame, stream=True, conf=0.1, verbose=False)

            annotated_frame = frame
            for r in results:
                # Tìm index của class 'crosswalk' trong danh sách label của model
                crosswalk_idx = None
                for idx, name in r.names.items():
                    if name.lower() == 'crosswalk':
                        crosswalk_idx = idx
                        break

                if crosswalk_idx is not None:
                    boxes = r.boxes
                    # Tìm index của crosswalk có confidence cao nhất
                    best_cw_idx = -1
                    max_conf = -1
                    
                    for i, box in enumerate(boxes):
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        if cls == crosswalk_idx:
                            if conf > max_conf:
                                max_conf = conf
                                best_cw_idx = i
                    
                    # Cập nhật r.boxes: chỉ giữ lại box crosswalk tốt nhất (nếu có)
                    if best_cw_idx != -1:
                        r.boxes = boxes[[best_cw_idx]]
                    else:
                        # Nếu không thấy crosswalk, xóa hết các detection khác để "chỉ detect được crosswalk"
                        r.boxes = boxes[[]]
                
                # Vẽ kết quả đã lọc lên frame
                annotated_frame = r.plot()

            # 4. Tính toán FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
            prev_time = curr_time
            
            # Ghi FPS lên màn hình
            cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 5. Hiển thị cửa sổ
            cv2.imshow("YOLO Model Test - Camera", annotated_frame)

            # Thoát nếu nhấn phím 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n🛑 Đã dừng theo yêu cầu người dùng.")
    finally:
        # Giải phóng tài nguyên
        cap.release()
        cv2.destroyAllWindows()
        print("👋 Cảm ơn bạn đã sử dụng!")

if __name__ == "__main__":
    main()
