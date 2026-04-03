import cv2
import numpy as np
import math

class LaneProcessor:
    def __init__(self, width, height, roi_factor=0.55):
        self.width = width
        self.height = height
        self.roi_top = int(height * roi_factor)
        self.roi_height = height - self.roi_top
        
        # EMA (Exponential Moving Average) alpha (0.0 to 1.0)
        # Higher = faster response, lower = smoother
        self.alpha = 0.2 
        self.smooth_heading = 0.0
        self.smooth_lateral = 0.0
        
        # Previous lines for fallback
        self.prev_left = None
        self.prev_right = None

    def get_roi(self, frame):
        """Extracts the lower part of the frame."""
        return frame[self.roi_top:self.height, :]

    def process_mask(self, mask):
        """
        Extracts left and right lane points by finding the largest contours 
        on the left and right sides of the image.
        """
        # Divide mask into left and right halves for contour analysis
        mid_x = self.width // 2
        mask_left = mask[:, :mid_x]
        mask_right = mask[:, mid_x:]

        def get_lane_pts(half_mask, offset_x=0):
            contours, _ = cv2.findContours(half_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            # Get the largest contour
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < 100: # Threshold to ignore tiny dots
                return None
            # Extract points and adjust by offset_x
            pts = largest.reshape(-1, 2)
            pts[:, 0] += offset_x # Shift X coordinates
            # Return in [y, x] format to match our polyfit logic
            return pts[:, [1, 0]] 

        left_pts = get_lane_pts(mask_left, 0)
        right_pts = get_lane_pts(mask_right, mid_x)

        left_line = self._fit_line_stable(left_pts)
        right_line = self._fit_line_stable(right_pts)

        # Fallback to previous if one is lost
        if left_line is None: left_line = self.prev_left
        if right_line is None: right_line = self.prev_right
        
        # Update history
        self.prev_left = left_line
        self.prev_right = right_line

        # Calculate center line
        center_line = self._get_center_line(left_line, right_line)

        return left_line, right_line, center_line

    def _fit_line_stable(self, pts):
        """Fits a line x = f(y) to points for stability with vertical lines."""
        if pts is None or len(pts) < 10:
            return None
            
        # We want x = my + c because lane lines are often near vertical (m -> infinity in y=mx+c)
        y = pts[:, 0]
        x = pts[:, 1]
        
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
        if left is None and right is None:
            return None
            
        if left is not None and right is not None:
            # Average the coordinates
            x1 = (left[0] + right[0]) // 2
            y1 = (left[1] + right[1]) // 2
            x2 = (left[2] + right[2]) // 2
            y2 = (left[3] + right[3]) // 2
            return (x1, y1, x2, y2)
        
        # Fallback if only 1 lane: shift by assumed lane width
        # Assumed half-lane width in pixels (tuning required)
        assumed_half_width = self.width // 4 
        
        if left is not None:
            return (left[0] + assumed_half_width, left[1], left[2] + assumed_half_width, left[3])
        
        if right is not None:
            return (right[0] - assumed_half_width, right[1], right[2] - assumed_half_width, right[3])

    def calculate_errors(self, center_line):
        """
        Calculates heading and lateral errors.
        Heading: degrees (-90 to 90), + is road turning RIGHT
        Lateral: pixels, + is lane center shifted to the RIGHT (vehicle is too far left)
        """
        if center_line is None:
            return 0.0, 0.0

        x_bot, y_bot, x_top, y_top = center_line
        
        # 1. Lateral Error: offset at the bottom of the frame
        # Vehicle is at self.width / 2
        lateral_error = x_bot - (self.width / 2)

        # 2. Heading Error: angle relative to vertical
        dx = x_top - x_bot
        dy = y_top - y_bot # Negative because y_top < y_bot
        
        # angle = atan2(dx, -dy) 
        # We use -dy to make "up" the positive axis
        heading_error_rad = math.atan2(dx, abs(dy))
        heading_error_deg = math.degrees(heading_error_rad)

        # Apply EMA smoothing
        self.smooth_heading = (self.alpha * heading_error_deg) + ((1 - self.alpha) * self.smooth_heading)
        self.smooth_lateral = (self.alpha * lateral_error) + ((1 - self.alpha) * self.smooth_lateral)

        return self.smooth_heading, self.smooth_lateral

    def draw_overlay(self, frame, left, right, center, heading, lateral):
        """Draws UI elements on the frame."""
        overlay = frame.copy()
        h, w = frame.shape[:2]
        mid_x = w // 2
        
        # Draw ROI boundary
        cv2.line(overlay, (0, self.roi_top), (w, self.roi_top), (0, 255, 255), 1)

        # Drawing helper
        def to_global(pt_roi):
            return (pt_roi[0], pt_roi[1] + self.roi_top)

        # Draw Lanes
        if left:
            cv2.line(overlay, to_global(left[:2]), to_global(left[2:]), (0, 0, 255), 3) # Red
        if right:
            cv2.line(overlay, to_global(right[:2]), to_global(right[2:]), (0, 255, 0), 3) # Green
        if center:
            cv2.line(overlay, to_global(center[:2]), to_global(center[2:]), (255, 0, 0), 2) # Blue
            
            # Draw lateral error indicator
            cv2.line(overlay, (mid_x, h-10), (int(center[0]), h-10), (255, 255, 0), 5)

        # Draw Vehicle Center Target
        cv2.circle(overlay, (mid_x, h-5), 8, (255, 255, 255), -1)
        cv2.line(overlay, (mid_x, h), (mid_x, h-50), (255, 255, 255), 2, cv2.LINE_AA)

        # Labels
        cv2.putText(overlay, f"Heading: {heading:.2f} deg", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(overlay, f"Lateral: {lateral:.2f} px", (20, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Visual hint for steering
        dir_text = "CENTERED"
        color = (0, 255, 0)
        if lateral > 20: 
            dir_text = "SHIFT RIGHT"
            color = (0, 165, 255)
        elif lateral < -20: 
            dir_text = "SHIFT LEFT"
            color = (0, 165, 255)
            
        cv2.putText(overlay, dir_text, (mid_x - 50, 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

        return overlay
