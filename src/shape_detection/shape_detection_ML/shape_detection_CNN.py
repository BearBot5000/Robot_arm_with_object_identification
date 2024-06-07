import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import tflite_runtime.interpreter as tflite

class ShapeDetectionNode(Node):
    #modified shape detection using A CNN tflite model
    def process_images(self):
        if self.top_down_image is not None and self.side_view_image is not None:
            combined_image = self.side_view_image.copy()
            combined_shapes = self.combine_results(self.top_down_shapes, self.side_view_shapes)
            if self.has_significant_change(combined_shapes):
                self.previous_combined_shapes = combined_shapes
                self.annotate_shapes(combined_image, combined_shapes, is_world=True)
                print(f"Updated Combined Shapes: {combined_shapes}")
            self.publish_detected_objects(combined_shapes)
            cv2.imshow('Top Down Camera', self.top_down_image)
            cv2.imshow('Side View Camera', self.side_view_image)
            cv2.imshow('Shape Detection Combined', combined_image)
            cv2.waitKey(1)
    
    def __init__(self):
        super().__init__('shape_detection_node')
        self.bridge = CvBridge()
        self.interpreter = tflite.Interpreter(model_path='path_to_your_shape_detection_model.tflite')
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Initialization
        print("ShapeDetectionNode initialized")

        # Publisher for detected objects
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

        # Camera parameters for determining the focal view
        self.camera_height = 2.5  # height in meters
        self.horizontal_fov = 1.047  # field of view in radians
        self.image_width = 1280  # width in pixels
        self.image_height = 960  # height in pixels
        self.world_width = 2 * self.camera_height * np.tan(self.horizontal_fov / 2)  # world width covered by the camera
        self.world_height = self.world_width * (self.image_height / self.image_width)  # world height covered by the camera
        # Offset adjustment
        self.offset_x = 0.00225476006211596
        self.offset_y = 0.00225476006211596

    def image_callback_top_down(self, msg):
        self.top_down_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.top_down_shapes = self.detect_shapes(self.top_down_image)
        self.process_images()

    def image_callback_side_view(self, msg):
        self.side_view_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.side_view_shapes = self.detect_shapes(self.side_view_image)
        self.process_images()

    

    def detect_shapes(self, input_image):
        shapes = []
        gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        resized_image = cv2.resize(gray, (self.input_details[0]['shape'][2], self.input_details[0]['shape'][1]))
        normalized_image = resized_image / 255.0
        input_data = np.expand_dims(normalized_image, axis=0).astype(np.float32)

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        preds = self.interpreter.get_tensor(self.output_details[0]['index'])

        for i in range(len(preds[0])):
            shape, color, cx, cy = preds[0][i]
            shapes.append((shape, color, int(cx), int(cy)))

        return shapes

    def combine_results(self, top_down_shapes, side_view_shapes):
        combined_shapes = []
        for top_shape, top_color, top_cx, top_cy in top_down_shapes:
            for side_shape, side_color, side_cx, side_cy in side_view_shapes:
                if abs(top_cx - side_cx) < 40:  
                    combined_shapes.append((top_color, top_shape, top_cx, top_cy))
        return combined_shapes

    def has_significant_change(self, current_shapes):
        if len(current_shapes) != len(self.previous_combined_shapes):
            return True
        for current, previous in zip(current_shapes, self.previous_combined_shapes):
            if current != previous:
                return True
        return False

    def annotate_shapes(self, image, shapes, is_world=False):
        for shape, color, cx, cy in shapes:
            label = f"{color} {shape} ({cx}, {cy})"
            cv2.putText(image, label, (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def publish_detected_objects(self, combined_shapes):
        detected_objects_json = json.dumps(combined_shapes)
        msg = String()
        msg.data = detected_objects_json
        self.publisher_.publish(msg)
        print(f"Published JSON: {detected_objects_json}")

def main(args=None):
    rclpy.init(args=args)
    shape_detection_node = ShapeDetectionNode()
    rclpy.spin(shape_detection_node)
    shape_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
