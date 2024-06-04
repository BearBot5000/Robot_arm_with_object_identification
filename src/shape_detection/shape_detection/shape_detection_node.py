import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ShapeDetectionNode(Node):
    def __init__(self):
        super().__init__('shape_detection_node')
        self.bridge = CvBridge()

        # Debugging initialization
        print("ShapeDetectionNode initialized")

        # Subscriptions for both cameras
        self.subscription_top_down = self.create_subscription(
            Image,
            '/top_down_camera/top_down_camera/image_raw',
            self.image_callback_top_down,
            10)
        self.subscription_top_down

        self.subscription_side_view = self.create_subscription(
            Image,
            '/side_view_camera/side_view_camera/image_raw',
            self.image_callback_side_view,
            10)
        self.subscription_side_view

        self.top_down_image = None
        self.side_view_image = None
        self.top_down_shapes = []
        self.side_view_shapes = []

    def image_callback_top_down(self, msg):
        self.top_down_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.top_down_shapes = self.detect_shapes(self.top_down_image, view="top")
        self.process_images()

    def image_callback_side_view(self, msg):
        self.side_view_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.side_view_shapes = self.detect_shapes(self.side_view_image, view="side")
        self.process_images()

    def process_images(self):
        if self.top_down_image is not None and self.side_view_image is not None:
            combined_image = self.top_down_image.copy()

            # Annotate shapes in the individual views
            self.annotate_shapes(self.top_down_image, self.top_down_shapes)
            self.annotate_shapes(self.side_view_image, self.side_view_shapes)
            
            # Combine the results from both views
            combined_shapes = self.combine_results(self.top_down_shapes, self.side_view_shapes)
            self.annotate_shapes(combined_image, combined_shapes)

            # Show each individual camera view and the combined view
            cv2.imshow('Top Down Camera', self.top_down_image)
            cv2.imshow('Side View Camera', self.side_view_image)
            cv2.imshow('Shape Detection Combined', combined_image)
            cv2.waitKey(1)

    def detect_shapes(self, input_image, view):
        hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        shapes = []

        for contour in contours:
            shape = self.detect_shape(contour, view)
            color = self.detect_color(contour, hsv)
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                shapes.append((shape, color, cx, cy))
        
        return shapes

    def detect_shape(self, contour, view):
        shape = "unidentified"
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

        if len(approx) == 3:
            shape = "triangle"
        elif len(approx) == 4:
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            if ar >= 0.95 and ar <= 1.05:
                shape = "cube"
            else:
                shape = "rectangle"
        else:
            area = cv2.contourArea(contour)
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            if hull_area > 0:
                solidity = float(area) / hull_area
                circularity = 4 * np.pi * area / (peri * peri)
                if len(approx) > 5 and solidity > 0.9 and circularity > 0.8:
                    if view == "top":
                        shape = "sphere"
                    else:
                        shape = "circle"
                else:
                    shape = "cylinder"
        return shape

    def detect_color(self, contour, hsv_image):
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        mean_val = cv2.mean(hsv_image, mask=mask)[:3]

        if mean_val[0] >= 0 and mean_val[0] <= 10:
            return "red"
        elif mean_val[0] >= 11 and mean_val[0] <= 25:
            return "orange"
        elif mean_val[0] >= 26 and mean_val[0] <= 34:
            return "yellow"
        elif mean_val[0] >= 35 and mean_val[0] <= 85:
            return "green"
        elif mean_val[0] >= 86 and mean_val[0] <= 125:
            return "blue"
        elif mean_val[0] >= 126 and mean_val[0] <= 145:
            return "purple"
        else:
            return "unidentified"

    def annotate_shapes(self, image, shapes):
        for shape, color, cx, cy in shapes:
            label = f"{color} {shape}"
            cv2.putText(image, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    def combine_results(self, top_down_shapes, side_view_shapes):
        combined_shapes = []

        for top_shape, top_color, top_cx, top_cy in top_down_shapes:
            for side_shape, side_color, side_cx, side_cy in side_view_shapes:
                if top_shape == "sphere" and side_shape == "circle":
                    combined_shapes.append((top_color, "sphere", top_cx, top_cy))
                elif top_shape == "circle" and side_shape == "cylinder":
                    combined_shapes.append((top_color, "cylinder", top_cx, top_cy))
                elif top_shape == "cube" and side_shape == "cube":
                    combined_shapes.append((top_color, "cube", top_cx, top_cy))
                elif top_shape == "rectangle" and side_shape == "cube":
                    combined_shapes.append((top_color, "cube", top_cx, top_cy))
                else:
                    combined_shapes.append((top_color, top_shape, top_cx, top_cy))
        
        return combined_shapes

def main(args=None):
    rclpy.init(args=args)
    shape_detection_node = ShapeDetectionNode()
    rclpy.spin(shape_detection_node)
    shape_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
