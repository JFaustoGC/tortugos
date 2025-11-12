import cv2
import numpy as np


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
    """Track robot position using OpenCV circle detection"""
    
    def __init__(self, camera_id=0, area_width_m=2.0, area_height_m=1.5, roi=None):
        """
        Args:
            camera_id: Camera device ID
            area_width_m: Real-world width of tracking area in meters
            area_height_m: Real-world height of tracking area in meters
            roi: Region of Interest as (x, y, width, height) in pixels, or None for full frame
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
        
        # Circle detection parameters (HSV color range for robot)
        # Default: detect any circular object
        self.min_radius = 10
        self.max_radius = 100
        
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
    
    def get_robot_position(self, show_debug=False):
        """
        Detect robot (circle) and return position in meters
        
        Args:
            show_debug: If True, display debug window with detection
        
        Returns:
            (x, y, detected): Position in meters from top-left corner, and detection flag
                             Returns (None, None, False) if not detected
        """
        if not self.cap or not self.cap.isOpened():
            return None, None, False
        
        # Read frame
        ret, frame = self.cap.read()
        if not ret:
            return None, None, False
        
        # Keep original frame for debug display
        original_frame = frame.copy() if show_debug else None
        
        # Apply ROI if specified
        if self.roi:
            roi_x, roi_y, roi_w, roi_h = self.roi
            frame = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        
        # Convert to grayscale for circle detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # Detect circles using Hough Circle Transform
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,
            param1=50,
            param2=30,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            # Take the first (most prominent) circle
            (x_px, y_px, radius) = circles[0]
            
            # Convert pixel coordinates to meters
            x_m = x_px / self.px_per_meter_x
            y_m = y_px / self.px_per_meter_y
            
            if show_debug:
                # Draw the circle and centroid on cropped frame
                cv2.circle(frame, (x_px, y_px), radius, (0, 255, 0), 2)
                cv2.circle(frame, (x_px, y_px), 3, (0, 0, 255), -1)
                
                # Add position text
                text = f"({x_m:.3f}m, {y_m:.3f}m)"
                cv2.putText(frame, text, (x_px - 50, y_px - radius - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # Show only the cropped ROI frame
                cv2.imshow("Robot Tracking", frame)
                cv2.waitKey(1)
            
            return x_m, y_m, True
        
        if show_debug:
            # Show only cropped frame when no detection
            cv2.imshow("Robot Tracking", frame)
            cv2.waitKey(1)
        
        return None, None, False
    
    def set_detection_params(self, min_radius=10, max_radius=100):
        """
        Configure circle detection parameters
        
        Args:
            min_radius: Minimum circle radius in pixels
            max_radius: Maximum circle radius in pixels
        """
        self.min_radius = min_radius
        self.max_radius = max_radius


if __name__ == "__main__":
    """Test the vision tracker"""
    import time
    
    # Initialize tracker directly with camera 4 (Logitech HD Pro Webcam C920)
    tracker = VisionTracker(
        camera_id=4,
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
                x, y, detected = tracker.get_robot_position(show_debug=True)
                
                if detected:
                    print(f"Robot position: X={x:.3f}m, Y={y:.3f}m")
                else:
                    print("Robot not detected")
                
                time.sleep(0.1)  # 10 Hz update rate
        
        except KeyboardInterrupt:
            print("\nStopping tracker...")
        finally:
            tracker.stop()
    else:
        print("Failed to start tracker")
