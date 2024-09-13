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
RES = (1280 , 800)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE,0)
tag_module = AprilTag()
ARUCO_LENGTH_METERS = 0.165
tag_module.calibrate(RES,'/Users/jaseer/Documents/GitHub/FRC2024-Offseason/src/main/python/charuco_images_jpeg',6,9,ARUCO_LENGTH_METERS/9,False)


def printAprilTagData(tagData):
    data = list(tagData.values())
    if len(data) > 0:
        print("Detected AprilTag " + str(tagData.keys())) # array of ints
        print('Translational: ' + str(data[0][0])) # array of double, "tag_transational" in nt
        print('Rotational: ' + str(data[0][1])) # array of double, "tag_rotational" in nt
        print('cvec: ' + str(data[0][2])) # double array, "tag_cvec" in nt


while True:
    data = list(tagData.values())

    p = time.time()
    try: 
        frame = input.getFrame()

        annotated_frame = frame.copy()
        tagData = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)
        printAprilTagData(tagData)
        annotated_frame = cv2.resize(annotated_frame, (320,240))
        
        pose_list = [4000 for _ in range(16 * 6)]
        for key, value in tagData.items():
            pose_list[(key - 1) * 6 : (key * 6)] = np.concatenate((value[0].flatten(), value[1].flatten()), axis=0).tolist()
            
        table = inst.getTable("datatable")

        # tag_keys = table.getIntegerArrayTopic("tag_keys").publish()
        # tag_keys.set(tagData.keys())

        # tag_transational = table.getDoubleArrayTopic("tag_transational").publish()
        # tag_transational.set(data[0][0])

        # tag_rotational = table.getDoubleArrayTopic("tag_rotational").publish()
        # tag_rotational.set(data[0][1])

        # tag_cvec = table.getDoubleArrayTopic("tag_cvec").publish()
        # tag_cvec.set(data[0][2])

        tag_yaw = table.getDoubleTopic("april_yaw").publish()
        tag_yaw.set(data[0][0][2])

        tag_id = table.getIntegerTopic("april_id").publish()
        tag_id.set(list(tagData.keys())[0])

        #xPub = table.getDoubleTopic("fps_incremented_value").publish()
        #xPub.set(frame.sum())

        tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
        tagDataPub.set(pose_list)
        
        # outputStreamPub = table.getDoubleArrayTopic("output_stream").publish()
        # outputStreamPub.set(annotated_frame.flatten().tolist())

        cv2.imshow('result', annotated_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        time.sleep(0.02)
    except KeyboardInterrupt:
        print("keyboard interrupt")
        input.close()
        break
    except Exception as e:
        print("An exception occurred:", e)
        #raise e
        
    #print('Loop time: ' + str(time.time()-p))