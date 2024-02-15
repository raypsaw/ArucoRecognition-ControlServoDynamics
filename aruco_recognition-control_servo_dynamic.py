# Import all library to run this program
import argparse
import imutils
from imutils.video import VideoStream
import time
import cv2
import sys
import numpy as np
import pyfirmata

# Initilaize board arduino (name port)
board = pyfirmata.Arduino('/dev/cu.usbserial-1410')

# Initialize pin board arduino for servo and relay
out_servo_x = board.get_pin('d:6:s')
out_servo_xx = board.get_pin('d:5:s')
out_servo_xxx = board.get_pin('d:3:s')
out_servo_y = board.get_pin('d:9:s')
out_servo_yy = board.get_pin('d:10:s')
out_servo_yyy = board.get_pin('d:11:s')
board.digital[8].write(1)

# Setup the pin of servo and initialize variable for servo
out_servo_x.write(90)
out_servo_xx.write(90)
out_servo_xxx.write(90)
out_servo_y.write(90)
out_servo_yy.write(90)
out_servo_yyy.write(90)
prev_servo_x = 90.0
prev_servo_y = 90.0
servo_x = 90.0
servo_y = 90.0
prev_error_x = 90.0
prev_error_y = 90.0
error_x = 90.0
error_y = 90.0

# Initialize gain for system control servo_x and servo_y using PID Controller
p_gain_x = 0.009
i_gain_x = 0.000001
d_gain_x = 0.0005
p_gain_y = 0.01
i_gain_y = 0.0000025
d_gain_y = 0.0005

# Initialize for integral and derrivative variabel for PID Controller
integral_x = 0.0
prev_error_x = 0.0
integral_y = 0.0
prev_error_y = 0.0

# Import the library for QR (Aruco)
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_4X4_50",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# Web for generate aruco : https://chev.me/arucogen/

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
vs = VideoStream(src=0).start()
time.sleep(0.3)

# Main Loop for this program
while True:
	# Run the camera
	frame = vs.read()
	frame = imutils.resize(frame, width=1000)
	
	# Object detection from library aruco and create box
	(corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
		arucoDict, parameters=arucoParams)
	
	if len(corners) > 0:
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			# Create box from each point in coordinate x,y
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			lengthTop = topRight[0] - topLeft[0]
			lengthBottom = bottomRight[0] - bottomLeft[0]
			lengthRight = bottomRight[1] - topRight[1]
			lengthLeft = bottomLeft[1] - topLeft[1]
			
			# Create circumference of a square
			totalRec = lengthTop + lengthBottom + lengthRight + lengthLeft
			
			# Design box
			cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
			
			# Create coordinate point of box
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
			cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			
			# Initialize each variable for maximum and minimum
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

			# Formula of Linear Regression for mapping variable
			map_servo_x = (((cX - min_cX) / (max_cX - min_cX)) * (max_servo_x - min_servo_x)) + min_servo_x
			map_servo_y = (((cY - min_cY) / (max_cY - min_cY)) * (max_servo_y - min_servo_y)) + min_servo_y
			distance = (((totalRec - min_totalRec) / (max_totalRec - min_totalRec)) * (max_distance - min_distance)) + min_distance
			
			# Undetection if QR to small, markerID 17, markerID 37
			if(totalRec <= 200 or markerID == 17 or markerID == 37):
				servo_x = prev_servo_x
				servo_y = prev_servo_y
				out_servo_x.write(servo_x)
				out_servo_xx.write(servo_x)
				out_servo_xxx.write(servo_x)
				out_servo_y.write(servo_y)
				out_servo_yy.write(servo_y)
				out_servo_yyy.write(servo_y)
			
			# Detection QR from library Aruco
			else:
				if(cX <= 498):
					error_x = 498 - cX
					integral_x += error_x
					derivative_x = error_x - prev_error_x
					p_term_x = p_gain_x * error_x
					i_term_x = i_gain_x * integral_x
					d_term_x = d_gain_x * derivative_x
					servo_x = prev_servo_x + (p_term_x + i_term_x + d_term_x)

				elif(cX >= 502):
					error_x = cX - 502
					integral_x += error_x
					derivative_x = error_x - prev_error_x
					p_term_x = p_gain_x * error_x
					i_term_x = i_gain_x * integral_x
					d_term_x = d_gain_x * derivative_x
					servo_x = prev_servo_x - (p_term_x + i_term_x + d_term_x)

				else:
					servo_x = servo_x
				
				if(cY <= 348):
					error_y = 348 - cY
					integral_y += error_y
					derivative_y = error_y - prev_error_y
					p_term_y = p_gain_y * error_y
					i_term_y = i_gain_y * integral_y
					d_term_y = d_gain_y * derivative_y
					servo_y = prev_servo_y - (p_term_y + i_term_y + d_term_y)

				elif(cY >= 352):
					error_y = cY - 352
					integral_y += error_y
					derivative_y = error_y - prev_error_y
					p_term_y = p_gain_y * error_y
					i_term_y = i_gain_y * integral_y
					d_term_y = d_gain_y * derivative_y
					servo_y = prev_servo_y + (p_term_y + i_term_y + d_term_y)

				else:
					servo_y = servo_y

				if(servo_x < max_servo_x):
					servo_x = max_servo_x
				elif(servo_x > min_servo_x):
					servo_x = min_servo_x
				else:
					servo_x = servo_x

				if(servo_y < max_servo_y):
					servo_y = max_servo_y
				elif(servo_y > min_servo_y):
					servo_y = min_servo_y
				else:
					servo_y = servo_y

				# Overwrite value of servo_x and servo_y from calculation above
				out_servo_x.write(int(servo_x))
				out_servo_xx.write(int(servo_x))
				out_servo_xxx.write(int(servo_x))
				out_servo_y.write(int(servo_y))
				out_servo_yy.write(int(servo_y))
				out_servo_yyy.write(int(servo_y))

				# Save the previous value for calculation
				prev_error_x = error_x
				prev_error_y = error_y
				prev_servo_x = servo_x
				prev_servo_y = servo_y

			# Display each value for correction
			print("ID       : ", markerID)
			print("cX       : ", int(cX))
			print("cY       : ", int(cY))
			print("Servo X  : ", int(servo_x))
			print("Servo Y  : ", int(servo_y))
			print("Distance : ", int(distance))
			print("----------------")
			
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	
	if key == ord("q"):
		break
	
cv2.destroyAllWindows()
vs.stop()