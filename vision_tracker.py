import cv2
import numpy as np
import math


def list_available_cameras(max_test=10):
    """
    Test camera indices to find available cameras
    
    Args:
        max_test: Maximum camera index to test
    
    Returns:
        List of available camera indices with resolution info
    """
    available = []
    for i in range(max_test):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                available.append(i)
                print(f"Camera {i}: Available - Resolution: {w}x{h}")
            else:
                print(f"Camera {i}: Opened but cannot read")
            cap.release()
        else:
            print(f"Camera {i}: Not available")
    
    return available


class VisionTracker:
    """Track robot position and orientation using ArUco markers"""
    
    def __init__(self, camera_id=0, area_width_m=2.0, area_height_m=1.5, roi=None, aruco_dict_type=cv2.aruco.DICT_4X4_50):
        """
        Args:
            camera_id: Camera device ID
            area_width_m: Real-world width of tracking area in meters
            area_height_m: Real-world height of tracking area in meters
            roi: Region of Interest as (x, y, width, height) in pixels, or None for full frame
            aruco_dict_type: ArUco dictionary type (default: DICT_4X4_50)
        """
        self.camera_id = camera_id
        self.area_width_m = area_width_m
        self.area_height_m = area_height_m
        self.roi = roi  # (x, y, width, height)
        
        # Calibration values (pixels per meter)
        self.px_per_meter_x = None
        self.px_per_meter_y = None
        
        # Camera capture
        self.cap = None
        
        # ArUco marker detection
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Target marker ID (None means detect any marker)
        self.target_marker_id = None
        
        # Reference position for display (optional)
        self.reference_position = None  # (x_ref, y_ref, theta_ref)
        
    def start(self):
        """Initialize camera and calibrate"""
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.camera_id}")
        
        # Try to set higher resolution for better tracking
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # Get frame dimensions for calibration
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read from camera")
        
        height_px, width_px = frame.shape[:2]
        
        # Verify what resolution we actually got
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        print(f"Requested: 1920x1080, Got: {actual_width}x{actual_height}")
        print(f"Frame shape: {width_px}x{height_px}")
        
        # If ROI is specified, use it for calibration
        if self.roi:
            roi_x, roi_y, roi_w, roi_h = self.roi
            print(f"Using ROI: x={roi_x}, y={roi_y}, w={roi_w}, h={roi_h}")
            # Calibrate based on ROI dimensions
            self.px_per_meter_x = roi_w / self.area_width_m
            self.px_per_meter_y = roi_h / self.area_height_m
        else:
            # Calibrate based on full frame
            self.px_per_meter_x = width_px / self.area_width_m
            self.px_per_meter_y = height_px / self.area_height_m
        
        print(f"Camera initialized: {width_px}x{height_px} pixels")
        print(f"Calibration: {self.px_per_meter_x:.2f} px/m (X), {self.px_per_meter_y:.2f} px/m (Y)")
        print(f"Tracking area: {self.area_width_m}m x {self.area_height_m}m")
        
        return True
    
    def stop(self):
        """Release camera"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
    
    def get_robot_position(self, show_debug=False) -> tuple:
        """
        Detect robot using ArUco marker and return position and orientation
        
        Args:
            show_debug: If True, display debug window with detection
        
        Returns:
            (x, y, theta, detected): Position in meters from top-left corner, 
                                     orientation in radians, and detection flag
                                     Returns (None, None, None, False) if not detected
        """
        # Orchestrator: read frame, detect markers, compute pose and optionally draw debug
        if not self.cap or not self.cap.isOpened():
            return None, None, None, False

        frame = self._read_frame()
        if frame is None:
            return None, None, None, False

        frame_roi = self._apply_roi(frame)
        gray = self._to_gray(frame_roi)

        corners, ids, rejected = self._detect_markers(gray)

        selected = self._select_marker(corners, ids)
        if selected is None:
            if show_debug:
                self._draw_debug(frame_roi, None, None, None, ids)
            return None, None, None, False

        marker_corners, marker_id = selected

        centroid_x, centroid_y, theta = self._compute_centroid_and_orientation(marker_corners)
        x_m, y_m = self._pixels_to_meters(centroid_x, centroid_y)

        if show_debug:
            self._draw_debug(frame_roi, (centroid_x, centroid_y), theta, marker_id, ids)

        return x_m, y_m, theta, True

    # --- Helper methods to make detection flow clearer ---
    def _read_frame(self):
        if not self.cap:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def _apply_roi(self, frame):
        if self.roi:
            roi_x, roi_y, roi_w, roi_h = self.roi
            return frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        return frame

    def _to_gray(self, frame):
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _detect_markers(self, gray):
        return self.aruco_detector.detectMarkers(gray)

    def _select_marker(self, corners, ids):
        # Return (marker_corners, marker_id) or None
        if ids is None or len(ids) == 0:
            return None

        # If a specific target is set, try to find it
        if self.target_marker_id is not None:
            marker_indices = np.where(ids == self.target_marker_id)[0]
            if len(marker_indices) == 0:
                return None
            idx = marker_indices[0]
        else:
            idx = 0

        marker_corners = corners[idx][0]
        marker_id = int(ids[idx][0]) if ids is not None else None
        return marker_corners, marker_id

    def _compute_centroid_and_orientation(self, marker_corners):
        centroid_x = float(np.mean(marker_corners[:, 0]))
        centroid_y = float(np.mean(marker_corners[:, 1]))

        # Use top edge to compute orientation (top-left -> top-right)
        top_left = marker_corners[0]
        top_right = marker_corners[1]
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]
        theta = math.atan2(dy, dx)
        return centroid_x, centroid_y, theta

    def _pixels_to_meters(self, centroid_x, centroid_y):
        x_m = (centroid_x / self.px_per_meter_x) - (self.area_width_m / 2.0)
        y_m = (centroid_y / self.px_per_meter_y) - (self.area_height_m / 2.0)
        return x_m, y_m

    def _draw_debug(self, frame, centroid, theta, marker_id, ids):
        # Draw detected markers and debug information on the given frame
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, None, ids)

        if centroid is not None:
            centroid_x, centroid_y = centroid
            cv2.circle(frame, (int(centroid_x), int(centroid_y)), 5, (0, 0, 255), -1)
            arrow_length = 50
            end_x = int(centroid_x + arrow_length * math.cos(theta))
            end_y = int(centroid_y + arrow_length * math.sin(theta))
            cv2.arrowedLine(frame, (int(centroid_x), int(centroid_y)),
                            (end_x, end_y), (0, 255, 0), 3, tipLength=0.3)

        # Draw reference position if available
        if self.reference_position is not None:
            x_ref_m, y_ref_m, theta_ref = self.reference_position
            x_ref_px = (x_ref_m + self.area_width_m / 2.0) * self.px_per_meter_x
            y_ref_px = (y_ref_m + self.area_height_m / 2.0) * self.px_per_meter_y
            ref_x = int(x_ref_px)
            ref_y = int(y_ref_px)
            cv2.drawMarker(frame, (ref_x, ref_y), (255, 0, 0), cv2.MARKER_CROSS, 20, 3)
            ref_arrow_length = 40
            ref_end_x = int(ref_x + ref_arrow_length * math.cos(theta_ref))
            ref_end_y = int(ref_y + ref_arrow_length * math.sin(theta_ref))
            cv2.arrowedLine(frame, (ref_x, ref_y), (ref_end_x, ref_end_y), (255, 0, 0), 2, tipLength=0.3)
            if centroid is not None:
                cv2.line(frame, (int(centroid[0]), int(centroid[1])), (ref_x, ref_y), (255, 255, 0), 1, cv2.LINE_AA)

        # Add position text if available
        if centroid is not None and marker_id is not None:
            x_m, y_m = self._pixels_to_meters(centroid[0], centroid[1])
            text = f"ID:{marker_id} ({x_m:.3f}m, {y_m:.3f}m, {math.degrees(theta):.1f}deg)"
            cv2.putText(frame, text, (int(centroid[0]) - 100, int(centroid[1]) - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow("Robot Tracking", frame)
        cv2.waitKey(1)
    
    def set_target_marker(self, marker_id: int = 0) -> None:
        """
        Set the target ArUco marker ID to track
        
        Args:
            marker_id: Marker ID to track, or 0 to track any marker
        """
        # Accept None to track any marker (default), otherwise store the desired id
        if marker_id is None:
            self.target_marker_id = None
        else:
            self.target_marker_id = int(marker_id)
    
    def set_reference_position(self, x_ref, y_ref, theta_ref):
        """
        Set reference position for visualization
        
        Args:
            x_ref: Reference x position in meters (centered coordinate system)
            y_ref: Reference y position in meters (centered coordinate system)
            theta_ref: Reference orientation in radians
        """
        self.reference_position = (x_ref, y_ref, theta_ref)


if __name__ == "__main__":
    """Test the vision tracker"""
    import time
    
    # Initialize tracker directly with camera 4 (Logitech HD Pro Webcam C920)
    tracker = VisionTracker(
        camera_id=0,
        area_width_m=2.45,   # 2.4 meters wide
        area_height_m=1.47,   # 1.45 meters tall
        roi=(360, 180, 1200, 720)  # ROI configuration
    )
    
    print(f"Using camera 4 (Logitech HD Pro Webcam C920)")
    print("Starting vision tracker...")
    if tracker.start():
        print("Camera ready. Press Ctrl+C to stop.")
        
        try:
            while True:
                x, y, theta, detected = tracker.get_robot_position(show_debug=True)
                
                if detected:
                    print(f"Robot position: X={x:.3f}m, Y={y:.3f}m, Theta={math.degrees(theta):.1f}°")
                else:
                    print("Robot not detected")
                
                time.sleep(0.1)  # 10 Hz update rate
        
        except KeyboardInterrupt:
            print("\nStopping tracker...")
        finally:
            tracker.stop()
    else:
        print("Failed to start tracker")
