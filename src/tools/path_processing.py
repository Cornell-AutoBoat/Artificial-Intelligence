import numpy as np
import math
from src import SFR
from src.tools.visualizations import path_visualizer
from test.msg import List, MotionPlans


def sgn(num):
    # helper function: sgn(num)
    # returns -1 if num is negative, 1 otherwise
    if num >= 0:
        return 1
    else:
        return -1


def inject_waypoints(path, segment_length):
    # injects waypoints into the path. Waypoints will be evenly spaced by segment_length
    # meters
    new_path = []

    for i in range(0, len(path)-1):

        distance = np.sqrt((path[i+1][0] - path[i][0])
                           ** 2 + (path[i+1][1] - path[i][1])**2)
        num_of_points = int(round(distance/segment_length))

        if num_of_points == 0:
            new_path.append(path[i])

        else:
            segment_x = (path[i+1][0] - path[i][0]) / num_of_points
            segment_y = (path[i+1][1] - path[i][1]) / num_of_points

            for j in range(0, num_of_points):
                new_point = [(path[i][0] + j*segment_x),
                             (path[i][1] + j*segment_y)]
                new_path.append(new_point)

    new_path.append(path[-1])

    return new_path


def find_min_angle(absTargetAngle, currentHeading):

    # minAngle = absTargetAngle - currentHeading
    minAngle = currentHeading - absTargetAngle

    if minAngle > np.pi or minAngle < -np.pi:
        minAngle = -1 * sgn(minAngle) * (2*np.pi - abs(minAngle))

    return minAngle


def smoothing(path, weight_data, weight_smooth, tolerance):
    # autoSmooth helper
    smoothed_path = path.copy()
    change = tolerance

    while change >= tolerance:
        change = 0.0

        for i in range(1, len(path)-1):

            for j in range(0, len(path[i])):
                aux = smoothed_path[i][j]

                smoothed_path[i][j] += weight_data * (path[i][j] - smoothed_path[i][j]) + weight_smooth * (
                    smoothed_path[i-1][j] + smoothed_path[i+1][j] - (2.0 * smoothed_path[i][j]))
                change += np.abs(aux - smoothed_path[i][j])

    return smoothed_path


def auto_smooth(path, maxAngle):
    # Returns a smoothed path of waypoints. maxAngle (rad) is the maximum change
    # in angle relative to the boat between waypoint segments
    currentMax = 0
    param = 0.01
    new_path = path
    firstLoop = True

    counter = 0

    while (currentMax >= maxAngle or firstLoop == True) and counter <= 15:

        param += 0.01
        firstLoop = False

        counter += 1
        # print('this is the {} iteration'.format(counter))

        new_path = smoothing(path, 0.1, param, 0.1)
        currentMax = 0

        for i in range(1, len(new_path)-2):
            angle1 = math.atan2(
                new_path[i][1] - new_path[i-1][1], new_path[i][0] - new_path[i-1][0])
            if angle1 < 0:
                angle1 += 2*np.pi
            angle2 = math.atan2(
                new_path[i+1][1] - new_path[i][1], new_path[i+1][0] - new_path[i][0])
            if angle2 < 0:
                angle2 += 2*np.pi

            if abs(find_min_angle(angle2, angle1)) > currentMax:
                currentMax = abs(find_min_angle(angle2, angle1))

    return new_path


def convert_to_msg_type(path):
    # ROS doesn't allow 2D lists as a message type. Therefore we have to convert the path to a list of List objects
    path_list = []
    for waypt in path:
        lst = List()
        lst.list = waypt
        path_list.append(lst)
    return path_list


def process(waypoints, fname="path.png"):
    """
    Preprocesses the list of waypoints in preparation for sending it to the controller. Performs waypoint injection and waypoint smoothing.
    Args:
        waypoints: 2D list (float64[][])
    Returns:
        path: 2D list (float64[][]), injected + smoothed version of waypoints
    """
    path = waypoints.copy()
    # Add the position of the boat to the start of the list of waypoints. Necessary for point injection and smoothing
    path.insert(0, [SFR.tx, SFR.ty])

    # Inject waypoints 1 meter apart
    path = inject_waypoints(path, 1)

    # Smooth the path of waypoints
    path = auto_smooth(path, np.pi/3)

    # Convert to List[]
    path = convert_to_msg_type(path)

    # UNCOMMENT FOR VISUALIZATIONS!! Change field size according to buoy placement
    # path_visualizer(waypoints, path, (1, 1), (-5, -5, 10, 10), fname)

    return path


def send_to_controls(motion_type, path=[]):
    """
    Publish a message to Controls containing the motion type, path, and current task being executed.
    Args:
        motion type: Must be one of the following strings: "path", "fwd", "bwd", "pivot_l", "pivot_r", "stop"
        path: 2D list in the form List[]
    """
    msg = MotionPlans()
    msg.motion_type = motion_type
    msg.path = path
    msg.task = SFR.task.name
    SFR.mcPub.publish(msg)
