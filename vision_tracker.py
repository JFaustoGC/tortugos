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
        
        # frame = cv2.flip(frame, -1) # rotate 180 degrees  
        
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
        if not self.cap or not self.cap.isOpened():
            return None, None, None, False
        
        # Read frame
        ret, frame = self.cap.read()
        if not ret:
            return None, None, None, False
        
        frame = cv2.flip(frame, -1)  # rotate 180 degrees
        
        # Apply ROI if specified
        if self.roi:
            roi_x, roi_y, roi_w, roi_h = self.roi
            frame = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if ids is not None and len(ids) > 0:
            # If target_marker_id is set, find that specific marker
            marker_idx = 0
            if self.target_marker_id is not None:
                marker_indices = np.where(ids == self.target_marker_id)[0]
                if len(marker_indices) == 0:
                    # Target marker not found
                    if show_debug:
                        cv2.imshow("Robot Tracking", frame)
                        cv2.waitKey(1)
                    return None, None, None, False
                marker_idx = marker_indices[0]
            
            # Get marker corners (4 corners of the marker)
            marker_corners = corners[marker_idx][0]
            
            # Calculate centroid (center of the marker)
            centroid_x = np.mean(marker_corners[:, 0])
            centroid_y = np.mean(marker_corners[:, 1])
            
            # Calculate orientation from marker corners
            # Use the vector from bottom-left to bottom-right corner
            # ArUco corners are ordered: top-left, top-right, bottom-right, bottom-left
            top_left = marker_corners[0]
            top_right = marker_corners[1]
            bottom_right = marker_corners[2]
            bottom_left = marker_corners[3]
            
            # Calculate orientation using top edge (points in robot's forward direction)
            dx = top_right[0] - top_left[0]
            dy = top_right[1] - top_left[1]
            top_left = marker_corners[0]
            top_right = marker_corners[1]

            # image-space vector
            top_left = marker_corners[0]
            top_right = marker_corners[1]
            vec_px = top_right - top_left  # dx, dy in pixels
            
            dx = vec_px[0]
            dy = -vec_px[1]  # flip y
            theta = math.atan2(dy, dx)
            theta = theta % (2 * math.pi)



            
            # Convert pixel coordinates to meters
            # Offset to center (0,0) in the middle of the image
            x_m = (centroid_x / self.px_per_meter_x) - (self.area_width_m / 2.0)
            # y_m = (centroid_y / self.px_per_meter_y) - (self.area_height_m / 2.0)
            y_m = -(centroid_y / self.px_per_meter_y) + (self.area_height_m / 2.0)

            
            if show_debug:
                # Draw the detected marker
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Draw centroid
                cv2.circle(frame, (int(centroid_x), int(centroid_y)), 5, (0, 0, 255), -1)
                
                # Draw orientation arrow
                # arrow_length = 50
                # end_x = int(centroid_x + arrow_length * math.cos(theta))
                # end_y = int(centroid_y + arrow_length * math.sin(theta))
                # cv2.arrowedLine(frame, (int(centroid_x), int(centroid_y)), 
                #               (end_x, end_y), (0, 255, 0), 3, tipLength=0.3)
                
                arrow_length = 50
                end_x = int(centroid_x + arrow_length * math.cos(theta))
                end_y = int(centroid_y - arrow_length * math.sin(theta))  # note minus to convert y-up → y-down
                cv2.arrowedLine(frame, (int(centroid_x), int(centroid_y)), (end_x, end_y), (0, 255, 0), 3, tipLength=0.3)

                
                # Draw reference position if available
                if self.reference_position is not None:
                    x_ref_m, y_ref_m, theta_ref = self.reference_position
                    # Convert reference from meters (centered) to pixels
                    x_ref_px = (x_ref_m + self.area_width_m / 2.0) * self.px_per_meter_x
                    y_ref_px = (y_ref_m + self.area_height_m / 2.0) * self.px_per_meter_y
                    
                    # Draw reference marker (blue cross)
                    ref_x = int(x_ref_px)
                    ref_y = int(y_ref_px)
                    cv2.drawMarker(frame, (ref_x, ref_y), (255, 0, 0), 
                                  cv2.MARKER_CROSS, 20, 3)
                    
                    # Draw reference orientation arrow (blue)
                    ref_arrow_length = 40
                    ref_end_x = int(ref_x + ref_arrow_length * math.cos(theta_ref))
                    ref_end_y = int(ref_y + ref_arrow_length * math.sin(theta_ref))
                    cv2.arrowedLine(frame, (ref_x, ref_y), 
                                  (ref_end_x, ref_end_y), (255, 0, 0), 2, tipLength=0.3)
                    
                    # Draw line connecting actual to reference
                    cv2.line(frame, (int(centroid_x), int(centroid_y)), 
                           (ref_x, ref_y), (255, 255, 0), 1, cv2.LINE_AA)
                
                # Add position and orientation text
                marker_id = ids[marker_idx][0]
                text = f"ID:{marker_id} ({x_m:.3f}m, {y_m:.3f}m, {math.degrees(theta):.1f}deg)"
                cv2.putText(frame, text, (int(centroid_x) - 100, int(centroid_y) - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                cv2.imshow("Robot Tracking", frame)
                cv2.waitKey(1)
            
            return x_m, y_m, theta, True
        
        if show_debug:
            cv2.imshow("Robot Tracking", frame)
            cv2.waitKey(1)
        
        return None, None, None, False
    
    def normalize_angle_0_to_2pi(self, angle):
        return angle % (2 * math.pi)


    
    def set_target_marker(self, marker_id: int = 0) -> None:
        """
        Set the target ArUco marker ID to track
        
        Args:
            marker_id: Marker ID to track, or 0 to track any marker
        """
        self.target_marker_id = marker_id
    
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
    
    print("Listing available cameras...")
    available_cams = list_available_cameras(max_test=10)
    print(f"Available cameras: {available_cams}")
    
    # Initialize tracker directly with camera 4 (Logitech HD Pro Webcam C920)
    tracker = VisionTracker(
        camera_id=0,
        area_width_m=2.4,   # 2.4 meters wide
        area_height_m=1.45,   # 1.45 meters tall
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
