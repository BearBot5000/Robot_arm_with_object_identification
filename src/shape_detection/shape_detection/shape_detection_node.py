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
            combined_image = self.side_view_image.copy()

            # Annotate shapes in the individual views
            self.annotate_shapes(self.top_down_image, self.top_down_shapes, (0, 0, 0))
            self.annotate_shapes(self.side_view_image, self.side_view_shapes, (0, 0, 0))
            
            # Combine the results from both views
            combined_shapes = self.combine_results(self.top_down_shapes, self.side_view_shapes)
            self.annotate_shapes(combined_image, combined_shapes, (0, 0, 0))

            # Show each individual camera view and the combined view
            cv2.imshow('Top Down Camera', self.top_down_image)
            cv2.imshow('Side View Camera', self.side_view_image)
            cv2.imshow('Shape Detection Combined', combined_image)
            cv2.waitKey(1)

    def detect_shapes(self, input_image, view):
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        shapes = []

        for contour in contours:
            shape = self.detect_shape(contour, view)
            color = self.detect_color(input_image, contour) if view == "top" else None
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
                elif view == "top":
                    shape = "circle"  # Detect circle when looking directly down on a cylinder
                else:
                    shape = "cylinder"
        return shape

    def detect_color(self, image, contour):
        mask = np.zeros(image.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [contour], -1, 255, -1)
        mean = cv2.mean(image, mask=mask)[:3]

        color_name = "unidentified"
        if mean[2] > 150 and mean[1] < 100 and mean[0] < 100:
            color_name = "red"
        elif mean[2] < 100 and mean[1] > 150 and mean[0] < 100:
            color_name = "green"
        elif mean[2] < 100 and mean[1] < 100 and mean[0] > 150:
            color_name = "blue"
        elif mean[2] > 150 and mean[1] > 150 and mean[0] < 100:
            color_name = "yellow"

        return color_name

    def annotate_shapes(self, image, shapes, text_color):
        for shape, color, cx, cy in shapes:
            if color is None:
                color = "unidentified"
            cv2.putText(image, f"{color} {shape}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)

    def combine_results(self, top_down_shapes, side_view_shapes):
        combined_shapes = []

        for side_shape, _, side_cx, side_cy in side_view_shapes:
            # Find the corresponding shape from the top-down view
            corresponding_top_shape = next((s for s in top_down_shapes if self.is_corresponding_shape(s, side_cx, side_cy)), None)
            #if corresponding_top_shape:
            #    _, top_color, _, _ = corresponding_top_shape
            #    combined_shapes.append((side_shape, top_color, side_cx, side_cy))
            #else:
            combined_shapes.append((side_shape, top_color, side_cx, side_cy))
        
        return combined_shapes

    def is_corresponding_shape(self, shape, x, y):
        _, _, shape_x, shape_y = shape
        # Adjust the distance threshold as needed to match corresponding shapes
        return abs(shape_x - x) < 20 and abs(shape_y - y) < 20

def main(args=None):
    rclpy.init(args=args)
    shape_detection_node = ShapeDetectionNode()
    rclpy.spin(shape_detection_node)
    shape_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
