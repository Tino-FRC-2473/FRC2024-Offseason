from vision_input import VisionInput
from apriltag import AprilTag
import time
import ntcore
import numpy as np
import cv2

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("python")
inst.setServerTeam(2473)

FOV = (50.28, 29.16)
RES = (640 , 480)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
# input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
# tag_module = AprilTag()
# ARUCO_LENGTH_METERS = 0.165

        table = inst.getTable("datatable")

while True:
    p = time.time()
    try: 
        frame = input.getFrame()

        annotated_frame = frame.copy()
        tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)
        annotated_frame = cv2.resize(annotated_frame, (320,240))
        
        pose_list = [4000 for _ in range(16 * 6)]
        for key, value in tagData.items():
             pose_list[(key - 1) * 6 : (key * 6)] = np.concatenate((value[0].flatten(), value[1].flatten()), axis=0).tolist()
            

        # xPub = table.getDoubleTopic("fps_incremented_value").publish()
        # xPub.set(frame.sum())

        tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
        # tagDataPub.set(pose_list)
        
        # outputStreamPub = table.getDoubleArrayTopic("output_stream").publish()
        # outputStreamPub.set(annotated_frame.flatten().tolist())
        inst = ntcore.NetworkTableInstance.getDefault()

        # Get the table within that instance that contains the data. There can
        # be as many tables as you like and exist to make it easier to organize
        # your data. In this case, it's a table called datatable.
        table = inst.getTable("datatable")

        # Start publishing topics within that table that correspond to the X and Y values
        # for some operation in your program.
        # The topic names are actually "/datatable/x" and "/datatable/y".
        xPub = table.getDoubleTopic("testX").publish()
        yPub = table.getDoubleTopic("testY").publish()

        x = 0
        y = 0

        while(True): 
            xPub.set(x)
            yPub.set(y)
            x += 0.05
            y += 1.0
            time.sleep(1)

        # cv2.imshow('result', annotated_frame)
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('q'):
        #     break
        # time.sleep(0.02)
    # except KeyboardInterrupt:
    #     print("keyboard interrupt")
    #     input.close()
    #     break
    except Exception as error:
         print("An exception occurred:", error)
    # print('Loop time: ' + str(time.time()-p))