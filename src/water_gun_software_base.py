"""
This file contains a commented version of the ZED Object Detection tutorial
"""

from distutils.command.bdist import show_formats
import pyzed.sl as sl
import cv2
import numpy as np
import serial
import time
 


def main():
    #START SERIAL COM
    serialCOM = serial.Serial(port='COM3', baudrate=9600, timeout=1)
    #############################################################################################################
    
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Set the depth mode to optimize performance (fastest)
    init_params.coordinate_units = sl.UNIT.METER # Sets position units in meters
    init_params.sdk_verbose = True # Sets verbose mode

    # Open the camera with the configuration parameters defined above
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS: # terminate the program if the camera initialization could not be completed
        exit(1)

    #############################################################################################################

    # Define the Objects detection module parameters
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking=True
    obj_param.image_sync=True
    obj_param.enable_mask_output=True

    # Object tracking requires the positional tracking module to be able to track the objects in the world
    # reference frame
    camera_infos = zed.get_camera_information()
    if obj_param.enable_tracking :
        positional_tracking_param = sl.PositionalTrackingParameters() # Set positional tracking parameters
        positional_tracking_param.set_floor_as_origin = True 
        zed.enable_positional_tracking(positional_tracking_param) # Enable positional tracking

    print("Object Detection: Loading Module...")

    ##############################################################################################################
    
    # Start the object detection module, which will load the trained CV model. 
    # The first time the module is used, the model will be optimized for the hardware and will take more time. 
    # This operation is done only once.

    # To choose a specific detection model by setting the detection_model field.
    # If none is specified then it defaults to MULTI_CLASS_BOX
    # obj_param.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX
    
    # Enable object detection with initialization parameters
    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS :
        print (repr(err))
        zed.close()
        exit(1)

    # The object detection module is now activated.

    ###############################################################################################################

    # The object confidence threshold can be adjusted at runtime to select only the revelant objects depending on
    # the scene complexity. Since the parameters have been set to image_sync, for each grab call, the image will 
    # be fed into the AI module and will output the detections for each frames.

    # To get the dectected objects in a scene, get an new image with grab(...) and extract the detected objects 
    # with retrieveObjects(). The objects' 2D positions are relative to the left image, while the 3D positions are
    # either in the CAMERA or WORLD reference frame depending on RuntimeParameters.measure3D_reference_frame,
    # which is given to the grab() function.

    # Detection Output
    objects = sl.Objects() # Structure containing all the detected objects

    # Define the detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40 # If the confidence in an object is over 40% it is a positive

    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        err = zed.retrieve_objects(objects, obj_runtime_param) # Retrieve the detected objects
        if objects.is_new : # Defines if the object list has already been retrieved or not
            
            # The sl.Objects class stores all the information regarding the different objects present in the scene in its object_list attribute
            # Each individual object is stored as a sl::ObjectData with all information about it, such as bounding box, position, mask, etc.
            obj_array = objects.object_list

            print(str(len(obj_array))+" Object(s) detected\n")

            # Prints out the object information for the first n objects.
            n = 1
            
            if len(obj_array) > 0 :
                position = [0,0,0]
                for obj in obj_array[:n]: # Iterate through the first n objects of the object list
                    print("Object attributes:")
                    print(" Label '" + repr(obj.label) + "' (conf. " + str(int(obj.confidence)) + "/100)")
                    if obj_param.enable_tracking :
                        print(" Tracking ID: " + str(int(obj.id)) + " tracking state: " + repr(obj.tracking_state) + " / " + repr(obj.action_state)) # 
                    position = obj.position
                    velocity = obj.velocity
                    dimensions = obj.dimensions

                    # Prints the x, y, and z vectors for position, velocity, and dimensions using a placeholder for   
                x = position[0]
                z = position[2]
                    # Placeholder Code
                value_to_move_DEG = np.arctan(x/z)
                i1_400 = (int((value_to_move_DEG/2)*1.8)//2)*2
                i = str(i1_400)
                input("enter to aim")
                if i == 'STOP':
                    print('Canceled Program')
                    break
                serialCOM.write(i.encode()) #Send string to Arduino
                status = 'Not Done'                
                while status != 'Done':
                    status = serialCOM.readline().decode('ascii') #Read string from Arduino
                    if status == 'ERROR':
                        print("Invalid Input")
                        break
                print(status)

                input("enter to shoot")
                if i == 'STOP':
                    print('Canceled Program')
                    break
                serialCOM.write(i.encode()) #Send string to Arduino
                status = 'Not Done'                
                while status != 'Done':
                    status = serialCOM.readline().decode('ascii') #Read string from Arduino
                    if status == 'ERROR':
                        print("Invalid Input")
                        break
                print(status)

    serialCOM.close()



    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()