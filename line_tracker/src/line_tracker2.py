#!/usr/bin/env python

"""
This script drives a simulated Turtlebot to follow lines around a Gazebo simulation.
The course containing lines to Turtlebot follow is illustrated in `tracking.png` file. 

The script uses ROSpy and OpenCV to extract the content of a `camera/rgb` ROS topic and
treats the image as a matrix of pixels.

Notes:
-----
- Be sure that you loaded the Gazebo environment of tracking running the command:

```
$ roslaunch trackingbot tracking.launch
```

- Turtlebot must subscribe to `camera/rgb/image_raw` topic to access images from camera
- Turtlebot must publish to cmd_vel_mux/input/teleop`  topic to move across the field

"""
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class TurtlebotLineTracker:

  def __init__(self):
    """ interface between ros and opencv """
    self.bridge = cv_bridge.CvBridge()
    # cv2.namedWindow("window", 1)

    # Subscribe to ROS topics
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub= rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    self.star = False


  def image_callback(self, data):
    """
    Callback of the ROS camera subscription
    
    `image_callback` should contain the code to:
      - identify the line in the image
      - calculate the center of the line (you may use the `cv2.moments()` function)
      - keep the line in the center of the image as the turtlebot moves
      - e.g. calculate the error between the center of the line and the
             center of the image. Publish the angular velocity as a 
             percentage of the error
    """
    # Read image from camera topic and transform to OpenCV format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except cv_bridge.CvBridgeError as e:
      print(e)
    
    height, width, channels = cv_image.shape
    # descentre = 160
    # rows_to_watch = 20
    # crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

    # Convert image from BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Apply a threshold for hues near the color yellow to obtain the binary image
    lower_yellow = numpy.array([20, 100, 100])
    upper_yellow = numpy.array([50, 255, 255])
    lower_blue = numpy.array([0, 100, 100])
    upper_blue = numpy.array([20, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    star_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    res = cv2.bitwise_and(hsv, cv_image, mask=mask)
    res_star = cv2.bitwise_and(hsv, cv_image, mask=star_mask)
    # res = cv2.bitwise_and(hsv, crop_img, mask=mask)

    # Find countours and centers for the line track
    contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    rospy.loginfo("Number of centroids: "+str(len(contours)))
    centers = []
    for i in range(len(contours)):
      moments = cv2.moments(contours[i])
      try:
        centers.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
        cv2.circle(res, centers[-1], 10, (0, 0, 255), -1)
      except ZeroDivisionError:
        pass
    
    # Find countours and centers for the star mask
    star_contours, _star = cv2.findContours(star_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    star_centers = []
    for i in range(len(star_contours)):
      moments = cv2.moments(star_contours[i])
      try:
        star_centers.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
        cv2.circle(res_star, star_centers[-1], 50, (255, 0, 0), -1)
      except ZeroDivisionError:
        pass

    # rospy.loginfo(str(centers))
    most_right_centroid_index = 0
    most_left_centroid_index = 0
    index = 0
    max_x_value = 0
    min_x_value = 0
    for center in centers:
      # Retrieve the cx value
      cx = centers[0]
      # Get the cx more to the right
      if cx >= max_x_value:
        max_x_value = cx
        most_right_centroid_index = index
      if cx <= min_x_value: # Or to the left
        min_x_value = cx
        most_left_centroid_index = index
      index += 1

    if len(star_centers) > 1:
      self.star = True

    if len(centers) < 1:
      linear = 0
    else:
      linear = 0.5

    try:
      if self.star == True:
        self.star == True
        print("ESQUERDA")
        cx = centers[most_left_centroid_index][0]
        cy = centers[most_left_centroid_index][1]
      else:
        print("DIREITA")
        cx = centers[most_right_centroid_index][0]
        cy = centers[most_right_centroid_index][1]
    except:
      cx, cy = height/2, width/2
    
    # Draw the centroid in the resultant image
    cv2.circle(res, (int(cx), int(cy)), 5, (0, 255, 0), -1)

    # Show images
    cv2.imshow("Original", cv_image)
    cv2.imshow("Star", res_star)
    cv2.imshow("Res", res)

    cv2.waitKey(1)

    # Calculating error to drive
    error_x = cx - width/2
    twist_object = Twist()
    twist_object.linear.x = linear
    twist_object.angular.z = -error_x / 100
    self.cmd_vel_pub.publish(twist_object)
    
  
  def clean_up(self):
    cv2.destroyAllWindows()

def main():
  rospy.init_node('turtlebot_line_tracker', anonymous=True)
  turtlebot_line_tracker = TurtlebotLineTracker()

  rate = rospy.Rate(5)
  ctrl_c = False
  def shutdown():
    turtlebot_line_tracker.clean_up()
    rospy.loginfo("Shutdown")
    ctrl_c = True
  
  while not ctrl_c:
    rate.sleep()

if __name__ == '__main__':
  main()
