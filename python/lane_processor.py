import cv2
import numpy as np
import math

class LaneProcessor:
    """
    Improved Lane Processor for indoor map lane centering.
    
    Key improvements over original:
    1. Dual ROI (near + far) for better lane tracking
    2. Row-wise midpoint scanning instead of raw centroid
    3. Morphological filtering to reject crosswalk / markings
    4. Aspect-ratio contour filtering to reject horizontal blobs
    5. Dual-error (lateral + heading) with separate smoothing
    6. Output rate limiting to reduce steering oscillation
    7. Curve-adaptive speed reduction
    """

    def __init__(self, width, height, roi_factor=0.55):
        self.width = width
        self.height = height
        
        # ===== ROI CONFIGURATION =====
        # TODO: Tune roi_near_top / roi_far_top for your camera angle
        # roi_near: bottom portion of frame – used for lateral error (bám giữa)
        # roi_far:  upper portion of ROI    – used for heading error (hướng lane)
        self.roi_top = int(height * roi_factor)           # top of full ROI
        self.roi_height = height - self.roi_top           # total ROI height
        # Near ROI = bottom 40% of the ROI region
        self.roi_near_top = int(self.roi_height * 0.6)    # TODO: Tune (0.5-0.7)
        # Far ROI = top 50% of the ROI region  
        self.roi_far_bottom = int(self.roi_height * 0.5)  # TODO: Tune (0.4-0.6)

        # ===== CONTOUR FILTERING =====
        # TODO: Tune min_contour_area to reject small noise
        self.min_contour_area = 150       # Minimum contour area in pixels
        # TODO: Tune max_aspect_ratio – crosswalk bars are wide & short (high aspect ratio)
        self.max_aspect_ratio = 4.0       # Reject blobs wider than 4x their height
        # TODO: Tune min_aspect_height – ignore very short blobs
        self.min_contour_height = 15      # Minimum bounding-box height in pixels

        # ===== LANE WIDTH ESTIMATION =====
        # TODO: Tune assumed_lane_width_px based on your camera FOV / map lane width
        self.assumed_lane_width_px = width // 2  # estimated full lane width in pixels

        # ===== SMOOTHING / FILTERING =====
        # EMA alpha for lane center coordinates (lower = smoother, higher = more responsive)
        # TODO: Tune alpha_center (0.1 = very smooth, 0.4 = responsive)
        self.alpha_center = 0.15

        # EMA alpha for final error outputs
        # TODO: Tune alpha_lateral, alpha_heading
        self.alpha_lateral = 0.20
        self.alpha_heading = 0.15

        # Rate limiter: max change per frame for steering-related errors
        # TODO: Tune max_lateral_delta, max_heading_delta
        self.max_lateral_delta = 15.0     # max px change per frame
        self.max_heading_delta = 5.0      # max deg change per frame

        # ===== CURVE SPEED REDUCTION =====
        # TODO: Tune heading_speed_threshold, min_speed_factor
        self.heading_speed_threshold = 10.0   # degrees – start reducing speed above this
        self.min_speed_factor = 0.45          # minimum speed multiplier at max curve

        # ===== INTERNAL STATE =====
        self.smooth_center_near_x = float(width // 2)  # smoothed near-ROI center X
        self.smooth_center_far_x = float(width // 2)    # smoothed far-ROI center X
        self.smooth_heading = 0.0
        self.smooth_lateral = 0.0
        self.prev_left = None
        self.prev_right = None
        self.prev_heading_raw = 0.0
        self.prev_lateral_raw = 0.0
        
        # Morphology kernels (pre-computed for performance)
        self._morph_close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        self._morph_open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # Vertical-emphasis kernel to suppress horizontal blobs (crosswalks)
        self._morph_vert_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9))

    # ------------------------------------------------------------------
    # PUBLIC API (same interface as original)
    # ------------------------------------------------------------------

    def get_roi(self, frame):
        """Extracts the lower part of the frame (unchanged interface)."""
        return frame[self.roi_top:self.height, :]

    def apply_morphology(self, mask):
        """
        [NEW] Apply morphological operations to clean the lane mask.
        Call this BEFORE process_mask() to reduce noise from crosswalks,
        markings, and small white blobs.
        """
        # 1. Close small gaps in lane lines
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_close_kernel)
        # 2. Remove small noise blobs
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_open_kernel)
        # 3. Vertical erosion to weaken horizontal features (crosswalks)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_vert_kernel)
        return mask

    def process_mask(self, mask):
        """
        Extracts left and right lane boundaries using row-wise scanning
        in near and far ROI regions, then computes a stable center line.
        
        Returns: (left_line, right_line, center_line)
            Each line = (x_bot, y_bot, x_top, y_top) in ROI coords, or None
        """
        mid_x = self.width // 2

        # --- Extract near & far ROI slices ---
        near_mask = mask[self.roi_near_top:, :]       # bottom portion
        far_mask  = mask[:self.roi_far_bottom, :]      # top portion

        # --- Find lane boundaries in each half (left / right) ---
        left_pts_near, right_pts_near = self._scan_lane_boundaries(near_mask, mid_x)
        left_pts_far, right_pts_far   = self._scan_lane_boundaries(far_mask, mid_x)

        # --- Merge near+far points for fitting ---
        left_pts = self._merge_points(left_pts_near, left_pts_far, y_offset_far=0, y_offset_near=self.roi_near_top)
        right_pts = self._merge_points(right_pts_near, right_pts_far, y_offset_far=0, y_offset_near=self.roi_near_top)

        # --- Also try contour-based extraction as fallback ---
        left_contour = self._find_lane_contour(mask[:, :mid_x], offset_x=0)
        right_contour = self._find_lane_contour(mask[:, mid_x:], offset_x=mid_x)

        # Merge contour points with scan points
        if left_pts is not None and left_contour is not None:
            left_pts = np.vstack([left_pts, left_contour])
        elif left_pts is None:
            left_pts = left_contour

        if right_pts is not None and right_contour is not None:
            right_pts = np.vstack([right_pts, right_contour])
        elif right_pts is None:
            right_pts = right_contour

        # --- Fit lines ---
        left_line = self._fit_line_stable(left_pts)
        right_line = self._fit_line_stable(right_pts)

        # Fallback to previous if lost
        if left_line is None:
            left_line = self.prev_left
        if right_line is None:
            right_line = self.prev_right

        # Update history
        self.prev_left = left_line
        self.prev_right = right_line

        # Calculate center line
        center_line = self._get_center_line(left_line, right_line)

        return left_line, right_line, center_line

    def calculate_errors(self, center_line):
        """
        Calculates heading and lateral errors with enhanced smoothing + rate limiting.
        
        Returns: (heading_deg, lateral_px, speed_factor)
            heading_deg: smoothed heading error in degrees (+ = road curving right)
            lateral_px:  smoothed lateral offset in pixels (+ = lane center right of vehicle)
            speed_factor: 0.0-1.0 multiplier for base speed (reduced on curves)
        """
        if center_line is None:
            # Decay toward zero when no lane is visible
            self.smooth_heading *= 0.9
            self.smooth_lateral *= 0.9
            return self.smooth_heading, self.smooth_lateral

        x_bot, y_bot, x_top, y_top = center_line

        # --- 1. Raw Lateral Error (near ROI – bottom of center line) ---
        raw_lateral = x_bot - (self.width / 2.0)

        # --- 2. Raw Heading Error (direction of center line) ---
        dx = x_top - x_bot
        dy = y_top - y_bot  # negative because y_top < y_bot
        heading_raw_rad = math.atan2(dx, abs(dy)) if abs(dy) > 1 else 0.0
        heading_raw_deg = math.degrees(heading_raw_rad)

        # --- 3. Rate Limiting (prevent sudden jumps) ---
        lateral_delta = raw_lateral - self.prev_lateral_raw
        heading_delta = heading_raw_deg - self.prev_heading_raw

        lateral_delta = max(-self.max_lateral_delta, min(self.max_lateral_delta, lateral_delta))
        heading_delta = max(-self.max_heading_delta, min(self.max_heading_delta, heading_delta))

        limited_lateral = self.prev_lateral_raw + lateral_delta
        limited_heading = self.prev_heading_raw + heading_delta

        self.prev_lateral_raw = limited_lateral
        self.prev_heading_raw = limited_heading

        # --- 4. EMA Smoothing ---
        self.smooth_lateral = (self.alpha_lateral * limited_lateral) + \
                              ((1 - self.alpha_lateral) * self.smooth_lateral)
        self.smooth_heading = (self.alpha_heading * limited_heading) + \
                              ((1 - self.alpha_heading) * self.smooth_heading)

        return self.smooth_heading, self.smooth_lateral

    def get_speed_factor(self):
        """
        [NEW] Returns a speed reduction factor (0.0 to 1.0) based on current
        heading error magnitude. Use this to slow down on curves.
        
        TODO: Tune heading_speed_threshold, min_speed_factor in __init__
        """
        abs_heading = abs(self.smooth_heading)
        if abs_heading <= self.heading_speed_threshold:
            return 1.0
        # Linear reduction from 1.0 down to min_speed_factor
        # over the range [threshold, 3*threshold]
        max_heading = self.heading_speed_threshold * 3.0
        factor = 1.0 - ((abs_heading - self.heading_speed_threshold) / 
                         (max_heading - self.heading_speed_threshold)) * (1.0 - self.min_speed_factor)
        return max(self.min_speed_factor, min(1.0, factor))

    def draw_overlay(self, frame, left, right, center, heading, lateral):
        """Draws UI elements on the frame (enhanced with dual-ROI visualization)."""
        overlay = frame.copy()
        h, w = frame.shape[:2]
        mid_x = w // 2

        # Draw ROI boundary
        cv2.line(overlay, (0, self.roi_top), (w, self.roi_top), (0, 255, 255), 1)
        
        # Draw near/far ROI boundaries within the ROI
        near_y_global = self.roi_top + self.roi_near_top
        far_y_global = self.roi_top + self.roi_far_bottom
        cv2.line(overlay, (0, near_y_global), (w, near_y_global), (255, 200, 0), 1)
        cv2.line(overlay, (0, far_y_global), (w, far_y_global), (200, 200, 255), 1)

        # Drawing helper
        def to_global(pt_roi):
            return (pt_roi[0], pt_roi[1] + self.roi_top)

        # Draw Lanes
        if left:
            cv2.line(overlay, to_global(left[:2]), to_global(left[2:]), (0, 0, 255), 3)  # Red
        if right:
            cv2.line(overlay, to_global(right[:2]), to_global(right[2:]), (0, 255, 0), 3)  # Green
        if center:
            cv2.line(overlay, to_global(center[:2]), to_global(center[2:]), (255, 0, 0), 2)  # Blue

            # Draw lateral error indicator
            cv2.line(overlay, (mid_x, h - 10), (int(center[0]), h - 10), (255, 255, 0), 5)

        # Draw Vehicle Center Target
        cv2.circle(overlay, (mid_x, h - 5), 8, (255, 255, 255), -1)
        cv2.line(overlay, (mid_x, h), (mid_x, h - 50), (255, 255, 255), 2, cv2.LINE_AA)

        # Labels
        speed_factor = self.get_speed_factor()
        cv2.putText(overlay, f"Heading: {heading:.2f} deg", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(overlay, f"Lateral: {lateral:.2f} px", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(overlay, f"Speed Factor: {speed_factor:.2f}", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # Visual hint for steering
        dir_text = "CENTERED"
        color = (0, 255, 0)
        if lateral > 20:
            dir_text = "SHIFT RIGHT"
            color = (0, 165, 255)
        elif lateral < -20:
            dir_text = "SHIFT LEFT"
            color = (0, 165, 255)

        cv2.putText(overlay, dir_text, (mid_x - 50, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

        return overlay

    # ------------------------------------------------------------------
    # PRIVATE HELPERS
    # ------------------------------------------------------------------

    def _scan_lane_boundaries(self, mask_slice, mid_x):
        """
        [NEW] Row-wise scan to find left-edge and right-edge of lane markings.
        Returns two arrays of (y, x) points, one for left boundary and one for right.
        This is more robust than centroid-based approaches because it finds the
        actual edges of lane markings row by row.
        """
        h, w = mask_slice.shape
        left_points = []
        right_points = []

        # Sample every N rows for performance (TODO: tune step for your resolution)
        step = max(1, h // 20)

        for row in range(0, h, step):
            row_data = mask_slice[row, :]
            white_cols = np.where(row_data > 128)[0]
            if len(white_cols) < 3:
                continue

            # Left half: find rightmost white pixel (= right edge of left lane marking)
            left_whites = white_cols[white_cols < mid_x]
            if len(left_whites) > 2:
                # Use the rightmost white pixel as the inner edge of the left lane line
                left_points.append([row, int(left_whites[-1])])

            # Right half: find leftmost white pixel (= left edge of right lane marking)
            right_whites = white_cols[white_cols >= mid_x]
            if len(right_whites) > 2:
                right_points.append([row, int(right_whites[0])])

        left_arr = np.array(left_points) if len(left_points) >= 3 else None
        right_arr = np.array(right_points) if len(right_points) >= 3 else None
        return left_arr, right_arr

    def _merge_points(self, pts_near, pts_far, y_offset_near, y_offset_far):
        """Merge near and far point arrays, adjusting Y coordinates to full ROI space."""
        adjusted = []
        if pts_near is not None and len(pts_near) >= 3:
            adj = pts_near.copy().astype(float)
            adj[:, 0] += y_offset_near
            adjusted.append(adj)
        if pts_far is not None and len(pts_far) >= 3:
            adj = pts_far.copy().astype(float)
            adj[:, 0] += y_offset_far
            adjusted.append(adj)
        if len(adjusted) == 0:
            return None
        return np.vstack(adjusted)

    def _find_lane_contour(self, half_mask, offset_x=0):
        """
        Contour-based lane point extraction (improved with aspect-ratio filtering).
        Returns points as Nx2 array [[y, x], ...] or None.
        """
        contours, _ = cv2.findContours(half_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Filter contours by area, aspect ratio, and height
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_contour_area:
                continue
            
            x, y, bw, bh = cv2.boundingRect(c)
            
            # Reject very short (horizontal) blobs – likely crosswalk bars
            if bh < self.min_contour_height:
                continue
            
            # Reject blobs with high width/height ratio – likely crosswalk / horizontal marking
            aspect = bw / max(bh, 1)
            if aspect > self.max_aspect_ratio:
                continue
            
            valid_contours.append(c)

        if not valid_contours:
            return None

        # Use the largest valid contour
        largest = max(valid_contours, key=cv2.contourArea)
        pts = largest.reshape(-1, 2)
        pts[:, 0] += offset_x  # shift X coords
        # Return as [y, x] for polyfit
        return pts[:, [1, 0]]

    def _fit_line_stable(self, pts):
        """Fits a line x = f(y) to points for stability with vertical lines."""
        if pts is None or len(pts) < 5:
            return None

        y = pts[:, 0].astype(float)
        x = pts[:, 1].astype(float)

        # Remove outliers using median-based filtering
        x_median = np.median(x)
        x_mad = np.median(np.abs(x - x_median)) + 1e-6
        # Keep points within 3 MAD of median
        inlier_mask = np.abs(x - x_median) < 3.0 * x_mad
        if np.sum(inlier_mask) < 5:
            return None
        y = y[inlier_mask]
        x = x[inlier_mask]

        try:
            # Polyfit degree 1: x = p[0]*y + p[1]
            p = np.polyfit(y, x, 1)

            # Bottom point (y = ROI height)
            y_bot = self.roi_height
            x_bot = int(p[0] * y_bot + p[1])

            # Top point (y = 0)
            y_top = 0
            x_top = int(p[0] * y_top + p[1])

            return (x_bot, y_bot, x_top, y_top)
        except:
            return None

    def _get_center_line(self, left, right):
        """Compute center line from left and right lane boundaries."""
        if left is None and right is None:
            return None

        if left is not None and right is not None:
            # Average the coordinates
            x1 = (left[0] + right[0]) // 2
            y1 = (left[1] + right[1]) // 2
            x2 = (left[2] + right[2]) // 2
            y2 = (left[3] + right[3]) // 2
            return (x1, y1, x2, y2)

        # Fallback if only 1 lane: shift by assumed half lane width
        assumed_half_width = self.assumed_lane_width_px // 2

        if left is not None:
            return (left[0] + assumed_half_width, left[1],
                    left[2] + assumed_half_width, left[3])

        if right is not None:
            return (right[0] - assumed_half_width, right[1],
                    right[2] - assumed_half_width, right[3])
