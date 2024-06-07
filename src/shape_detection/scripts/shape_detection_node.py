import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json

class ShapeDetectionNode(Node):
    def __init__(self):
        super().__init__('shape_detection_node')
        self.bridge = CvBridge()

        #initialization
        print("ShapeDetectionNode initialized")

        #publisher for detected objects
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)

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
        self.previous_combined_shapes = []

        #camera parameters for determining the focal view 
        self.camera_height = 2.5  # height in meters
        self.horizontal_fov = 1.047  # field of view in radians
        self.image_width = 1280  # width in pixels
        self.image_height = 960  # height in pixels
        self.world_width = 2 * self.camera_height * np.tan(self.horizontal_fov / 2)  # world width covered by the camera
        self.world_height = self.world_width * (self.image_height / self.image_width)  # world heigth covered by the camera
        #offset adjustment
        self.offset_x = 0.00225476006211596
        self.offset_y = 0.00225476006211596

    def image_callback_top_down(self, msg):
        self.top_down_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.top_down_shapes = self.detect_shapes(self.top_down_image, view="top")
        self.process_images()

    def image_callback_side_view(self, msg):
        self.side_view_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.side_view_shapes = self.detect_shapes(self.side_view_image, view="side")
        self.process_images()

    #controlling function for determining the combined shape and displaying the images
    def process_images(self):
        if self.top_down_image is not None and self.side_view_image is not None:
            combined_image = self.side_view_image.copy()
            #annotate shapes in the individual views
            self.annotate_shapes(self.top_down_image, self.top_down_shapes, is_world=False)
            self.annotate_shapes(self.side_view_image, self.side_view_shapes, is_world=False)
            #combine the results from both views
            combined_shapes = self.combine_results(self.top_down_shapes, self.side_view_shapes)
            #only update and print if there is a significant change
            if self.has_significant_change(combined_shapes):
                self.previous_combined_shapes = combined_shapes
                self.annotate_shapes(combined_image, combined_shapes, is_world=True)
                print(f"Updated Combined Shapes: {combined_shapes}")
            self.publish_detected_objects(combined_shapes)
            #resize the windows
            cv2.namedWindow('Top Down Camera', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Top Down Camera', 640, 480)  # Resize to 640x480
            cv2.namedWindow('Side View Camera', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Side View Camera', 640, 480)  # Resize to 640x480
            #cv2.namedWindow('Shape Detection Combined', cv2.WINDOW_NORMAL)
           # cv2.resizeWindow('Shape Detection Combined', 640, 480)  # Resize to 640x480
            #show top, side and combined camera (combined is side view)
            cv2.imshow('Top Down Camera', self.top_down_image)
            cv2.imshow('Side View Camera', self.side_view_image)
            #cv2.imshow('Shape Detection Combined', combined_image)
            cv2.waitKey(1)

    #check to see if there have been enough change to print an update
    def has_significant_change(self, current_shapes):
        if len(current_shapes) != len(self.previous_combined_shapes):
            return True

        for current, previous in zip(current_shapes, self.previous_combined_shapes):
            if current != previous:
                return True
        return False

    #math to determine the shape
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
                #print(f"Detected {color} {shape} at ({cx}, {cy}) in {view} view")
        return shapes
    #more math for shape detection (assigns shape based off of contour)
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

    #detect the colors of the shape
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

    def annotate_shapes(self, image, shapes, is_world=False):
        for shape, color, cx, cy in shapes:
            if is_world:
                label = f"{color} {shape} ({cx:.2f}, {cy:.2f})"
            else:
                label = f"{color} {shape} ({cx}, {cy})"
            cv2.putText(image, label, (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    #convert the pixels to world <x,y> coords
    def pixel_to_world(self, cx, cy, image_width, image_height):
        #adjust the scaling factor to map pixel distances to world distances
        scale_x = self.world_width / image_width
        scale_y = self.world_height / image_height
        #center coordinates (639, 479)
        center_x = image_width / 2
        center_y = image_height / 2
        #convert pixel coordinates to world coordinates inverted for camera rotation
        world_x = -(cx - center_x) * scale_x - self.offset_x
        world_y = (cy - center_y) * scale_y - self.offset_y  
        return world_x, world_y

    #combine images, determine shapes, set color
    def combine_results(self, top_down_shapes, side_view_shapes):
        combined_shapes = []
        image_height, image_width, _ = self.top_down_image.shape
        for top_shape, top_color, top_cx, top_cy in top_down_shapes:
            matched = False
            for side_shape, side_color, side_cx, side_cy in side_view_shapes:
                if abs(top_cx - side_cx) < 40:  # Adjust threshold as needed
                    world_x, world_y = self.pixel_to_world(top_cx, top_cy, image_width, image_height)
                    if top_shape == "sphere" and side_shape == "circle":
                        combined_shapes.append((top_color, "sphere", world_x, world_y))
                    elif top_shape == "circle" and side_shape == "cylinder":
                        combined_shapes.append((top_color, "cylinder", world_x, world_y))
                    elif top_shape == "sphere" and side_shape == "cylinder":
                        combined_shapes.append((top_color, "cylinder", world_x, world_y))
                    elif top_shape == "sphere" and side_shape == "cube":
                        combined_shapes.append((top_color, "cylinder", world_x, world_y))
                    elif top_shape == "cube" and side_shape == "cube":
                        combined_shapes.append((top_color, "cube", world_x, world_y))
                    elif top_shape == "rectangle" and side_shape == "cube":
                        combined_shapes.append((top_color, "cube", world_x, world_y))
                    elif top_shape == "rectangle" and side_shape == "rectangle":
                        combined_shapes.append((top_color, "cube", world_x, world_y))
                    matched = True
                    break
            if not matched:
                world_x, world_y = self.pixel_to_world(top_cx, top_cy, image_width, image_height)
                combined_shapes.append((top_color, top_shape, world_x, world_y))      
        return combined_shapes

    #print the detected shapes and colors to json for use pick and place operations
    def publish_detected_objects(self, combined_shapes):
        #convert the combined_shapes list to a JSON string
        detected_objects_json = json.dumps(combined_shapes)
        msg = String()
        msg.data = detected_objects_json
        self.publisher_.publish(msg)
        #print(f"Published JSON: {detected_objects_json}")

# main
def main(args=None):
    rclpy.init(args=args)
    shape_detection_node = ShapeDetectionNode()
    rclpy.spin(shape_detection_node)
    shape_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
