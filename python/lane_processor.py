import cv2
import numpy as np


class LaneProcessor:
    """
    Simplified Lane Processor – Boundary Detection Only.
    
    Logic:
    1. Detect yellow border using HSV color mask
    2. Draw a horizontal scan line across the camera view
    3. If yellow pixels cross the scan line center → boundary detected
       (car is approaching the edge of the map)
    4. Send boundary flag to STM32 → STM32 handles: straight 2s → turn right → straight
    """

    def __init__(self, width, height, roi_factor=0.55):
        self.width = width
        self.height = height
        self.roi_top = int(height * roi_factor)
        self.roi_height = height - self.roi_top

        # ===== SCAN LINE CONFIGURATION =====
        # Position of horizontal scan line within ROI (0.0 = top of ROI, 1.0 = bottom)
        # Lower value = detect further ahead, Higher = closer to car
        # TODO: Tune scan_line_ratio for your camera angle (try 0.2 – 0.5)
        self.scan_line_ratio = 0.35
        self.scan_line_y = int(self.roi_height * self.scan_line_ratio)

        # Only check the CENTER portion of the scan line
        # This avoids false positives from yellow borders on the sides of the road
        # TODO: Tune scan_margin (0.0 = full width, 0.3 = ignore 30% each side)
        self.scan_margin = 0.20
        self.scan_x_start = int(width * self.scan_margin)
        self.scan_x_end = int(width * (1.0 - self.scan_margin))

        # Required fill ratio to trigger boundary (0.0 to 1.0)
        # 0.85 means 85% of the scan line must be yellow
        # This ensures it ONLY triggers when the whole line is crossing, not just a corner
        self.min_fill_ratio = 0.85

        # Morphology kernels for noise reduction
        self._morph_close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self._morph_open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def get_roi(self, frame):
        """Extracts the lower part of the frame."""
        return frame[self.roi_top:self.height, :]

    def apply_morphology(self, mask):
        """Clean the mask with morphological operations."""
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_open_kernel)
        return mask

    def detect_boundary(self, mask):
        """
        Check if yellow pixels fill most of the horizontal scan line.
        
        Returns: (boundary_detected: bool, fill_ratio: float)
        """
        scan_row = mask[self.scan_line_y, self.scan_x_start:self.scan_x_end]
        yellow_count = int(np.count_nonzero(scan_row))
        
        # Calculate how many percent of the scan area is yellow
        total_pixels = self.scan_x_end - self.scan_x_start
        fill_ratio = yellow_count / total_pixels if total_pixels > 0 else 0
        
        boundary_detected = fill_ratio >= self.min_fill_ratio
        return boundary_detected, fill_ratio

    def draw_overlay(self, frame, boundary_detected, fill_ratio):
        """Draw scan line and boundary status on frame."""
        overlay = frame.copy()
        h, w = frame.shape[:2]

        # Draw ROI top boundary
        cv2.line(overlay, (0, self.roi_top), (w, self.roi_top), (0, 255, 255), 1)

        # Draw scan line: GREEN = clear, RED = boundary detected
        scan_y_global = self.roi_top + self.scan_line_y
        color = (0, 0, 255) if boundary_detected else (0, 255, 0)
        thickness = 3 if boundary_detected else 2

        # Active scan region (center)
        cv2.line(overlay, (self.scan_x_start, scan_y_global),
                 (self.scan_x_end, scan_y_global), color, thickness)
        # Inactive margins (dim gray)
        cv2.line(overlay, (0, scan_y_global),
                 (self.scan_x_start, scan_y_global), (100, 100, 100), 1)
        cv2.line(overlay, (self.scan_x_end, scan_y_global),
                 (w, scan_y_global), (100, 100, 100), 1)

        # Status text
        status = "!! BOUNDARY !!" if boundary_detected else "CLEAR"
        cv2.putText(overlay, f"{status} ({fill_ratio*100:.1f}%)", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        return overlay
