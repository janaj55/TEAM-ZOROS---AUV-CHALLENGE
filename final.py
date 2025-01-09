
#AUV-230283

# We used *Blue* color box instead of *Red* due to availablity 
# all colour tuning are done underwater and may very upon different environment conditions
# we use dronekit library

# Notes :

# 3 rd RC channel - Vertical control of AUV
# 4 th RC channel - Yaw control of AUV
# 5 th RC channel - Foward / Backward motion control of AUV
# 6 th RC channel - Lateral motion of AUV
# 8 th RC channel - Servo is connected to FC

import time
import cv2
import numpy as np
from dronekit import * 

vehicle = connect("/dev/ttyAMA0",baud=921600, wait_ready=True) # FC UART PORT

def arm():
    print("Basic pre-arm checks")


    print("Arming motors")

    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

known_width=15
focal_length=833

def calculate_depth(obtained_width,known_width,focal_length):
	return(known_width*focal_length)/obtained_width

text=True
tune=False
a=1
l_kp=0.4#.05
l_ki=0
l_kd=0
l_integral=0
l_error=0
l_prev_error=0
servo_open=0
y_kp=8
y_ki=0
y_kd=0
y_integral=0
y_error=0
y_prev_error=0


t_integral=0
t_error=0
t_prev_error=0
t_prev_error_b=0
t_integral_b=0

t_error_b=0
y_prev_error_b,y_integral_b,y_error_b=0,0,0
f_pwm=1580
desired_depth=60.00#cm

d_pwm=1535
dt=0.01
kp_ll=0
ap_ll=0
drop_counter=0
black_detected=0
blue_detected=0
orange_detected=0
drop=0
vehicle.channels.overrides['8'] = 2000 #make sure servo is the closed position

l_error_b,l_prev_error_b,l_integral_b=0,0,0
def lateral_pid(l_error,l_previous_error,l_integral):
	p_term=l_kp*l_error
	l_integral=l_integral+l_error*dt
	i_term=l_ki*l_integral
	derivative=(l_error-l_prev_error)/dt
	d_term=l_kd*derivative
	output=1500+p_term+i_term+d_term
	output=max(1000,min(2000,output))
	return int(output),l_integral

def yaw_pid(y_error,y_previous_error,y_integral):
	p_term=y_kp*y_error
	y_integral=y_integral+y_error*dt
	i_term=y_ki*y_integral
	derivative=(y_error-y_prev_error)/dt
	d_term=y_kd*derivative
	output=1500+p_term+i_term+d_term
	output=max(1000,min(2000,output))
	return int(output),y_integral

def throttle_pid(t_error,t_previous_error,t_integral):
	global d_pwm
	if t_error<0:
		t_kp=1
		t_ki=0
		t_kd=0
	elif t_error>=0:
		t_kp=4
		t_ki=0
		t_kd=0
	p_term=t_kp*t_error
	t_integral=t_integral+t_error*dt
	i_term=t_ki*t_integral
	derivative=(t_error-t_prev_error)/dt
	d_term=t_kd*derivative
	output=1535+p_term+i_term+d_term
	output=max(1000,min(2000,output))
	output=int(output)
	if 1450<=output<=1564:
		output=d_pwm

	return output,t_integral

# Callback function for the trackbars

def nothing(x):
    pass
if tune==True:
	# Create a window for trackbars
	cv2.namedWindow("Trackbars")
	cv2.createTrackbar("Lower H", "Trackbars", 0, 180, nothing)
	cv2.createTrackbar("Lower S", "Trackbars", 66, 255, nothing)
	cv2.createTrackbar("Lower V", "Trackbars", 0, 255, nothing)
	cv2.createTrackbar("Upper H", "Trackbars", 180, 180, nothing)
	cv2.createTrackbar("Upper S", "Trackbars", 197, 255, nothing)
	cv2.createTrackbar("Upper V", "Trackbars", 255, 255, nothing)
	cv2.createTrackbar("Erode Iter", "Trackbars", 7, 10, nothing)
	cv2.createTrackbar("Dilate Iter", "Trackbars", 10, 10, nothing)



cap = cv2.VideoCapture(0)  # Use the first connected webcam
result = cv2.VideoWriter('filename.avi', 
						cv2.VideoWriter_fourcc(*'MJPG'), 
						10, (640, 360)) 
time.sleep(0.1)

x_last = 320 
y_last = 180

while cap.isOpened():
	if a==0:
		arm()
		vehicle.mode=VehicleMode("STABILIZE")
		#print("set all controls zero")
		vehicle.channels.overrides['3'] = 1500 # Vertical control of AUV
		vehicle.channels.overrides['4'] = 1500 # Yaw control of AUV
		vehicle.channels.overrides['5'] = 1500 # Forward/backward control of AUV
		vehicle.channels.overrides['6'] = 1500 # Lateral control of AUV
		a=1
	start_time=time.time()
	ret, frame = cap.read()
	if not ret:
		#print("Failed to grab frame. Exiting...")
		break	
	image = frame
	image = cv2.resize(image, (640, 360))

    # Get HSV values from the sliders
	if tune==True:
		lh = cv2.getTrackbarPos("Lower H", "Trackbars")
		ls = cv2.getTrackbarPos("Lower S", "Trackbars")
		lv = cv2.getTrackbarPos("Lower V", "Trackbars")
		uh = cv2.getTrackbarPos("Upper H", "Trackbars")
		us = cv2.getTrackbarPos("Upper S", "Trackbars")
		uv = cv2.getTrackbarPos("Upper V", "Trackbars")
		# Get erosion and dilation iterations from sliders
		erode_iter = cv2.getTrackbarPos("Erode Iter", "Trackbars")
		dilate_iter = cv2.getTrackbarPos("Dilate Iter", "Trackbars")
	else:
		erode_iter=0
		dilate_iter=4
		lh, ls, lv=00,00,40
		uh, us, uv=92,91,172
  
	# Convert the frame to HSV
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
  # Threshold the HSV image to get the black line

	blackbox= cv2.inRange(hsv, (0,0,0), (60,60,60))
	kernel = np.ones((3,3), np.uint8)
	blackbox = cv2.erode(blackbox, kernel, iterations=erode_iter)
	blackbox = cv2.dilate(blackbox, kernel, iterations=dilate_iter)	
	contours_black, hierarchy_black = cv2.findContours(blackbox.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.imshow("orginal", blackbox)
	min_contour_area_black = 150000  # Adjust this value as per your requirements
	contours_black_len = len(contours_black)

	if contours_black_len > 0 :
		for j in range(contours_black_len):# >= 1 :

			#(x_box,y_box) = blue_box			
			contour_area_black = cv2.contourArea(contours_black[j])
					#print("count_a :",contour_area)
			if contour_area_black < min_contour_area_black:
						
						black_detected=0
						continue  # Skip small contours
			else:
				a=0 # initiate arming and change vehicle to STABILIZE mode 
       
       
 # Threshold the HSV image to get the Orange line
	Orangeline = cv2.inRange(hsv, (lh, ls, lv), (uh, us, uv))
	Orangeline = cv2.erode(Orangeline, kernel, iterations=erode_iter)
	Orangeline = cv2.dilate(Orangeline, kernel, iterations=dilate_iter)	
	contours_org, hierarchy_org = cv2.findContours(Orangeline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.imshow("orginal", Orangeline)
	cv2.drawContours(image,contours_org,-1,(0,255,0),3)

 # Threshold the HSV image to get the blue line

	bluebox= cv2.inRange(hsv, (46, 113, 162), (136, 197, 255))
	bluebox = cv2.erode(bluebox, kernel, iterations=erode_iter)
	bluebox = cv2.dilate(bluebox, kernel, iterations=dilate_iter)	
	contours_blue, hierarchy_blue = cv2.findContours(bluebox.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.imshow("orginal", Orangeline)
	min_contour_area_blue = 150000  # Adjust this value as per your requirements
	contours_blue_len = len(contours_blue)

	if contours_blue_len > 0 :
		for j in range(contours_blue_len):

			contour_area_blue = cv2.contourArea(contours_blue[j])
					
			if contour_area_blue < min_contour_area_blue:
						
						blue_detected=0
						continue  # Skip small contours
			else:
				bluebox = cv2.minAreaRect(contours_blue[j])
				(x_min_b, y_min_b), (w_min_b, h_min_b), ang_b = bluebox
				if text==True:	
					cv2.putText(image,"w_min_b:" +str(round(w_min_b,2)),(300, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
					cv2.putText(image,"h_min_b:" +str(round(h_min_b,2)),(300, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)	
				blue_box = cv2.boxPoints(bluebox)
				blue_detected=1

				bluebox = np.int64(blue_box)
				cv2.drawContours(image,[bluebox],0,(0,0,255),3)	 

				if 50000<int(contour_area_blue)<500000:# these values are determined in the field testing

					drop=1

						
	min_contour_area_org = 10000  # Adjust this value as per your requirements

	contours_org_len = len(contours_org)
	if contours_org_len > 0 and drop==0 :
		if contours_org_len == 1 :
			orangebox = cv2.minAreaRect(contours_org[0])
			#print("orangebox1")
		else:
			canditates=[]
			off_bottom = 0	   
			for con_num in range(contours_org_len):	
				contour_area = cv2.contourArea(contours_org[con_num])
				#print("count_a :",contour_area)
				if contour_area < min_contour_area_org:
					continue  # Skip small contours
				else:

					orange_detected=1
					orangebox = cv2.minAreaRect(contours_org[con_num])
					(x_min, y_min), (w_min, h_min), ang = orangebox		
					box = cv2.boxPoints(orangebox)
					(x_box,y_box) = box[0]
					if y_box > 358 :		 
						off_bottom += 1
					canditates.append((y_box,con_num,x_min,y_min))		
			contours_org_len=len(canditates)
			canditates = sorted(canditates)
			if off_bottom > 1:	
				#print("off_bottom")    
				canditates_off_bottom=[]
				for con_num in range ((contours_org_len - off_bottom), contours_org_len):
					(y_highest,con_highest,x_min, y_min) = canditates[con_num]		
					total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
					canditates_off_bottom.append((total_distance,con_highest))
				canditates_off_bottom = sorted(canditates_off_bottom)         
				(total_distance,con_highest) = canditates_off_bottom[0]         
				orangebox = cv2.minAreaRect(contours_org[con_highest])	   
			else:
				if not canditates:
					continue  # Skip to the next frame or loop iteration
				else:

					(y_highest,con_highest,x_min, y_min) = canditates[contours_org_len-1]
			
					orangebox = cv2.minAreaRect(contours_org[con_highest])	 
		(x_min, y_min), (w_min, h_min), ang = orangebox


		x_last = x_min
		y_last = y_min
		p_ang=ang
		if ang < -45 :
			ang = 90 + ang
			
		if w_min < h_min and ang > 0:	  
			ang = (90-ang)*-1
			
		if w_min > h_min and ang < 0:
			ang = 90 + ang
			

		ang = int(ang)	 
		if ang<0:
			y_error=90+ang
		elif ang>0:
			y_error=ang-90
		
		y_pwm,y_integral=yaw_pid(y_error,y_prev_error,y_integral)
		if w_min>h_min:
			w_min,h_min=h_min,w_min

		setpoint = 320

		l_error = int(x_min - setpoint) #lateral channel 6
		l_pwm,l_integral=lateral_pid(l_error,l_prev_error,l_integral)

		box = cv2.boxPoints(orangebox)
		box = np.int64(box)
		cv2.drawContours(image,[box],0,(0,0,255),3)	 
		cv2.line(image, (int(x_min),200), (int(x_min),250), (255,0,0),3)


		vehicle.channels.overrides['4'] = y_pwm
		
		vehicle.channels.overrides['6'] = l_pwm
		if -45<l_error>45:
			vehicle.channels.overrides['5'] = f_pwm


		l_prev_error=l_error
		y_prev_error=y_error

	
		current_depth=calculate_depth(w_min,known_width,focal_length)
		if current_depth>60.0:
			current_depth=60.0
		t_error=(desired_depth-round(current_depth,2))
		t_pwm,t_integral=throttle_pid(t_error,t_prev_error,t_integral)
		t_prev_error=t_error


		if text==True:


			#cv2.putText(image,"l_e :"+str(l_error),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
			cv2.putText(image,"l_pwm : "+str(l_pwm),(10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

			#cv2.putText(image,"p_ang:" +str(round(p_ang,2)),(300, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

			cv2.putText(image,"y_e:" +str(y_error),(10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			cv2.putText(image,"y_pwm : "+str(y_pwm),(10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			cv2.putText(image,"depth_c :"+str(round(current_depth,2)),(10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
			#cv2.putText(image,"depth_e :"+str(round(t_error,2)),(10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
	else:
		orange_detected=0
		if drop==1 :


			current_depth=calculate_depth(w_min_b,known_width,focal_length)
			t_error_b=(desired_depth-current_depth)
			t_pwm,t_integral_b=throttle_pid(t_error,t_prev_error_b,t_integral_b)
			t_prev_error_b=t_error_b
			setpoint = 320


			l_error_b = int(x_min_b - setpoint) #lateral channel 6
			l_pwm  ,l_integral_b=lateral_pid(l_error_b,l_prev_error_b,l_integral_b)
			l_prev_error_b=l_error_b
			p_ang_b=ang_b
			if ang_b < -45 :
				ang_b = 90 + ang_b
				
			if w_min_b < h_min_b and ang_b > 0:	  
				ang_b = (90-ang_b)*-1
				
			if w_min_b > h_min_b and ang_b < 0:
				ang_b = 90 + ang_b
				

			ang_b = int(ang_b)	 #yaw channel 4
			if ang_b<0:
				y_error_b=90+ang_b
			elif ang_b>0:
				y_error_b=ang_b-90
			y_pwm,y_integral_b=yaw_pid(y_error_b,y_prev_error_b,y_integral_b)
			y_prev_error_b=y_error_b
			


			if text==True:
				cv2.putText(image,"l_e_b :"+str(l_error_b),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
				cv2.putText(image,"l_pwm_b : "+str(l_pwm),(10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

				cv2.putText(image,"p_amg_b:" +str(round(p_ang_b,2)),(300, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

				cv2.putText(image,"y_e/amg_b:" +str(y_error_b),(10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
				cv2.putText(image,"y_pwm_b : "+str(y_pwm),(10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)


				cv2.putText(image,"depth_c :"+str(round(current_depth,2)),(10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
				cv2.putText(image,"depth_e :"+str(round(t_error_b,2)),(10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
			if 250<int(w_min_b)<700 and 250<int(h_min_b)<700:
				t_pwm=d_pwm
				drop_counter+=1
				#print("incrementing counter")
			if drop_counter>3 and servo_open==0:
				vehicle.channels.overrides['5']=1500 # stop forward motion find center or drop

				vehicle.channels.overrides['8'] = 1000 # marker deployed
				print("----------------------marker dropeed----------------------")

				
				servo_open=1
			if servo_open==0:
				if -45<l_error_b>45:
					vehicle.channels.overrides['5'] = 1550
				vehicle.channels.overrides['3']=t_pwm
				vehicle.channels.overrides['6']=l_pwm
				vehicle.channels.overrides['4']=y_pwm
			elif servo_open==1:
				vehicle.channels.overrides['3']=1545
				vehicle.channels.overrides['6']=1500
				vehicle.channels.overrides['4']=1500
								


		else:
			drop_counter=0
			
			t_pwm=d_pwm
		
	vehicle.channels.overrides['3'] = t_pwm
	lidar_depth=vehicle.rangefinder.distance
	if text==True:
		cv2.putText(image,"t_pwm: "+str(t_pwm),(10, 280), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
	cv2.putText(image,"orng :"+str(orange_detected),(10, 310), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 128, 255), 2)
	#cv2.putText(image,"blk :"+str(black_detected),(150, 310), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
	#cv2.putText(image,"blue :"+str(blue_detected),(250, 310), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
	loop_time=time.time()-start_time
	#print("loop_time: ",loop_time)
	result.write(image) 

	cv2.imshow("orginal with line", image)	


	key = cv2.waitKey(1) & 0xFF	
	if key == ord("q"):
		vehicle.armed=False
		vehicle.channels.overrides['8'] = 1000
		result.release() 
		cap.release()

		break
