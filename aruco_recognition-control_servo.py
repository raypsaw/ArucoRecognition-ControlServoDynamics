import argparse
import imutils
from imutils.video import VideoStream
import time
import cv2
import sys
import numpy as np
import pyfirmata

board = pyfirmata.Arduino('/dev/cu.usbserial-1410')

out_servo_x = board.get_pin('d:10:s')
out_servo_y = board.get_pin('d:11:s')
board.digital[9].write(1)
prev_servo_x = .0
prev_servo_y = .0

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_4X4_50",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)
	
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

print("[INFO] starting video stream...")
# FRAME_W = 640
# FRAME_H = 480
# FRAME_RATE = 5
# vs = cv2.VideoCapture(0)
# vs.set(3, FRAME_W)
# vs.set(4, FRAME_H)
# vs.set(5, FRAME_RATE)
vs = VideoStream(src=0).start()
time.sleep(0.3)

while True:
	frame = vs.read()
	frame = imutils.resize(frame, width=1000)
	
	(corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
		arucoDict, parameters=arucoParams)
	
	if len(corners) > 0:
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			lengthTop = topRight[0] - topLeft[0]
			lengthBottom = bottomRight[0] - bottomLeft[0]
			lengthRight = bottomRight[1] - topRight[1]
			lengthLeft = bottomLeft[1] - topLeft[1]
			totalRec = lengthTop + lengthBottom + lengthRight + lengthLeft
			
			cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
			
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
			cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			max_cX = 900.0
			min_cX = 100.0
			max_cY = 50.0
			min_cY = 650.0
			max_totalRec = 2500.0
			min_totalRec = 100.0
			max_servo_x = 30.0
			min_servo_x = 150.0
			max_servo_y = 30.0
			min_servo_y = 150.0
			max_distance = 20.0
			min_distance = 100.0
			servo_x = (((cX - min_cX) / (max_cX - min_cX)) * (max_servo_x - min_servo_x)) + min_servo_x
			servo_y = (((cY - min_cY) / (max_cY - min_cY)) * (max_servo_y - min_servo_y)) + min_servo_y
			distance = (((totalRec - min_totalRec) / (max_totalRec - min_totalRec)) * (max_distance - min_distance)) + min_distance
			if(totalRec <= 200):
				servo_x = prev_servo_x
				servo_y = prev_servo_y
				out_servo_x.write(servo_x)
				out_servo_y.write(servo_y)
			else:
				prev_servo_x = servo_x
				prev_servo_y = servo_y
				out_servo_x.write(int(servo_x))
				out_servo_y.write(int(servo_y))
			print("ID : ", markerID)
			print("Servo X : ", int(servo_x))
			print("Servo Y : ", int(servo_y))
			print("Distance: ", int(distance))
			print("-----------------------")
			
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	
	if key == ord("q"):
		break
	
cv2.destroyAllWindows()
vs.stop()