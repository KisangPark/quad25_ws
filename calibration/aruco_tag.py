"""
things to study
1) argparse? -> get arguments and parse
"""


import cv2
import argparse

import numpy as np
from cv2 import aruco

#load calibration data -> saved from calibration.py
inner_matrix = np.load("matrix.npy")
distortion = np.load("distortion.npy")
real_size = 15

#load aruco dictionary, 16 binary!
arucodict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)



def main():

    #marker detection -> initialize parameters for detector
    detector_param = aruco.DetectorParameters_create()

    #get arguments, parse the video
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--video', type=str, default="http://192.168.168.105:8080/stream?topic=/argus/ar0234_rear/image_raw")
    parser.add_argument('--frame_name', type=str, default='front_right')
    parser.add_argument('--output', type=str, default="output.mp4")
    parser.add_argument('--codec', type=str, default="mp4v")
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=540)
    args = parser.parse_args()

    # Open the video from http server
    cap = cv2.VideoCapture(args.video)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error opening video stream or file")

    # Read until video is completed
    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret:
            #gray frame, detect marker
            #gray frame
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #return corners
            corners, ID, reject = aruco.detectMarkers(frame_gray, arucodict, parameters=detector_param)
            #if corners, estimate
            if corners:
                i=0
                rotation, transition, _ = aruco.estimatePoseSingleMarkers(corners, real_size, inner_matrix, distortion)

                for id, corner in zip(ID, corners):
                    cv2.polylines(frame, [corner.astype(np.int32)], True, (0,255,255), 4, cv2.LINE_AA)
                    
                    corner = corner.reshape(4,2)
                    corner = corner.astype(int)
                    top_right = corner[0].ravel() #np.ravel() -> contiguous flattened array!
                    top_left = corner[1].ravel()
                    bottom_right = corner[2].ravel()
                    bottom_left = corner[3].ravel()

                    #distance from the camera
                    distance = np.sqrt(transition[i][0][0]**2 + transition[i][0][1]**2 + transition[i][0][2]**2)

                    cv2.putText(frame, f"distance: {distance}", top_right, cv2.FONT_HERSHEY_PLAIN, 2.0, (200,100,0), 2, cv2.LINE_AA,)
                    i+=1

            # Display the resulting frame
            cv2.imshow(args.frame_name, frame)

            # Press Q on keyboard to exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        else:
            break

    # When everything done, release the video capture and writer objects
    cap.release()

    # Closes my window
    cv2.destroyWindow(args.frame_name)


if __name__ == '__main__':
    main()
