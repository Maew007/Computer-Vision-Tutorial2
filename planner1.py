import rospy
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String  
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

command_pub = rospy.Publisher("motor_commands",String)


def is_yellow(val):
	ref_color_yellow = 100
	return val > ref_color_yellow


def plan(left, right):
	command = "STOP"
	if is_yellow(left) and is_yellow(right):
		command = "GO"
	if is_yellow(left) and not is_yellow(right):
		command = "LEFT"
	if not is_yellow(left) and is_yellow(right):
		command = "RIGHT"
		
	print(left,right,command)
	command_pub.publish(command)
	#print("Plan sent sensor")
	return command
	

def plan5sensor(LLL,LL,L,C,CC,R,RR,RRR):
	command = "STOP"		
#	if not is_yellow(LL) and is_yellow(L)and is_yellow(C)and is_yellow(R)and not is_yellow(RR):
#		command = "GOM"
#	if is_yellow(LL) and is_yellow(L)and is_yellow(C)and is_yellow(R)and is_yellow(RR):
#		command = "GOM"
	if is_yellow(LL) and is_yellow(L)and is_yellow(C)and is_yellow(R)and is_yellow(RR):
		command = "GO"
	if is_yellow(LL) and is_yellow(L)and not is_yellow(C)and not is_yellow(R)and not is_yellow(RR):
		command = "LEFTL"
	if is_yellow(LL) and is_yellow(L)and is_yellow(C)and not is_yellow(R)and not is_yellow(RR):
		command = "LEFTL"
	if is_yellow(LL) and is_yellow(L)and is_yellow(C)and is_yellow(R)and not is_yellow(RR):
		command = "LEFTL"
	if not is_yellow(LL) and not is_yellow(L)and not is_yellow(C)and is_yellow(R)and is_yellow(RR):
		command = "RIGHTL"
	if not is_yellow(LL) and not is_yellow(L)and is_yellow(C)and is_yellow(R)and is_yellow(RR):
		command = "RIGHTL"
	if not is_yellow(LL) and is_yellow(L)and is_yellow(C)and is_yellow(R)and is_yellow(RR):
		command = "RIGHTL"
			
	#if not is_yellow(left) and is_yellow(right):
	#	command = "RIGHT"
			
	print(LL,L,C,R,RR,"Command = >",command)
	command_pub.publish(command)
	#print("Plan sent sensor")
	return command


def imgCallback(data):
	min_line = 200
	max_line = 500
	pos_ll = 250
	pos_l =  300
	pos_c =  350
	pos_cc = 350
	pos_r =  400
	pos_rr = 450
	pos_rrr = 550
	pos_lll = 150
        
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	#print(cv_image.shape)  #width = 800  Hight = 800
	gray_image =  cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
	#command = plan(gray_image[700][300],gray_image[700][500])
	data_s_ll = gray_image[700][pos_ll]
	data_s_l  = gray_image[700][pos_l]
	data_s_c  = gray_image[700][pos_c]
	data_s_r  = gray_image[700][pos_r]
	data_s_rr = gray_image[700][pos_rr]
	
	data_s_lll = gray_image[700][pos_lll]
	data_s_rrr = gray_image[700][pos_rrr]
	data_s_cc  = gray_image[500][pos_cc]
	
	command = plan5sensor(data_s_lll,data_s_ll,data_s_l,data_s_c,data_s_cc,data_s_r,data_s_rr,data_s_rrr)
	
	#3+@*I#@G2#'2A'-@G@-#L
	
	gray_image =  cv2.line(gray_image,(min_line+125,500),(max_line-125,500),1,5)
	
	gray_image =  cv2.line(gray_image,(min_line,700),(max_line,700),0,5)
	
	gray_image =  cv2.line(gray_image,(max_line+50,700),(max_line+100,700),1,5)
	gray_image =  cv2.line(gray_image,(min_line-50,700),(min_line-100,700),1,5)
	
	#3+3A+H@G@-#L@*I# 3A+H LLL 
	gray_image = cv2.circle(gray_image,(pos_lll,700),5,255, 1)
	#3+3A+H@G@-#L@*I# 3A+H LL 
	gray_image = cv2.circle(gray_image,(pos_ll,700),5,255, 1)
	#3+3A+H@G@-#L@*I# 3A+H L
	gray_image = cv2.circle(gray_image,(pos_l,700),5,255, 1)
	#3+3A+H@G@-#L@*I# 3A+H C
	gray_image = cv2.circle(gray_image,(pos_c,700),5,255, 1)
	#3+3A+H@G@-#L@*I# 3A+H CC
	gray_image = cv2.circle(gray_image,(pos_cc,500),5,255, 1)
	#3+3A+H@G@-#L@*I# 3A+H R
	gray_image = cv2.circle(gray_image,(pos_r,700),5,255, 1)
	#3+3A+H@G@-#L@*I# 3A+H RR
	gray_image = cv2.circle(gray_image,(pos_rr,700),5,255, 1)
	#3+3A+H@G@-#L@*I# 3A+H RRR
	gray_image = cv2.circle(gray_image,(pos_rrr,700),5,255, 1)
	
	if command == "LEFTH":	
		gray_image = cv2.circle(gray_image,(pos_rr,700),5,255, 2)
		gray_image = cv2.circle(gray_image,(pos_r,700),5,255, 2)
		gray_image = cv2.circle(gray_image,(pos_c,700),5,255, 2)
	if command == "LEFTM":	
		gray_image = cv2.circle(gray_image,(pos_rr,700),5,255, 2)
		gray_image = cv2.circle(gray_image,(pos_r,700),5,255, 2)
	if command == "LEFTL":	
		gray_image = cv2.circle(gray_image,(pos_rr,700),5,255, 2)
	if command == "GOH":	
		gray_image = cv2.circle(gray_image,(pos_c,700),5,255, 2)
	if command == "GOM":	
		gray_image = cv2.circle(gray_image,(pos_c,700),5,255, 2)
	if command == "GOL":	
		gray_image = cv2.circle(gray_image,(pos_c,700),5,255, 2)
	if command == "RIGHTH":	
		gray_image = cv2.circle(gray_image,(pos_ll,700),5,255, 2)
		gray_image = cv2.circle(gray_image,(pos_l,700),5,255, 2)
		gray_image = cv2.circle(gray_image,(pos_c,700),5,255, 2)
	if command == "RIGHTM":	
		gray_image = cv2.circle(gray_image,(pos_ll,700),5,255, 2)
		gray_image = cv2.circle(gray_image,(pos_l,700),5,255, 2)
	if command == "RIGHTL":	
		gray_image = cv2.circle(gray_image,(pos_ll,700),5,255, 2)	
		
	cv2.imshow("Raw Image", gray_image)
	#(thresh, blackAndWhiteImage)) = cv2.threshold(gray_image,127,255,cv2.
	cv2.waitKey(3)

def main():
  print("Hi Maew Team Robot Chandrakasem")
  rospy.init_node('my_planner_node')
  img_sub = rospy.Subscriber("/camera/image_raw",Image,imgCallback)
  rospy.spin()
if __name__ == "__main__":
  main()


