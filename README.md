# Computer-Vision-Tutorial2

Demo Computer vision control simple robot with ros noetic. 

This demo reference from web 
https://youtu.be/2-Taue1Ue_8?si=viRXmzNfrnZRRNgD

Befor use this code 
1. please follow  step by step from  https://youtu.be/2-Taue1Ue_8?si=viRXmzNfrnZRRNgD
2. cd ~/catkin_ws/
3. source ~/.bashrc
4. source ~/catkin_ws/devel/setup.bash
5. cd ~/catkin_ws/src/Computer_Vision_Tutorial/src
6. Edit file planner1.py use command $nano planner1.py
   #เริ่มต้นไฟล์
#นำเข้า คลังคำสั่งที่เกี่ยวข้อในการใช้งาน
#คลังคำสั่งที่เกี่ยวข้องกับระบบปฏิบัติการหุ่นยนต์
import rospy
#คลังคำสั่งที่เกี่ยวข้องกับการประมวลผลภาพ OpenCV
import cv2


#นำเข้าแบบเลือกบางฟังก์ชัน หรือตัวแปร ที่ต้องการเท่านั้น เพื่อลดจำนวนหน่วยความจำแรมที่ใช้ขณะทำงาน

#นำเข้าแบบเลือก ตัวแปรชื่อ Image จาก Class ชื่อ sensor_msgs.msg 
from sensor_msgs.msg import Image
#นำเข้าแบบเลือก ตัวแปรชื่อ String จาก Class ชื่อ sensor_msgs.msg
from std_msgs.msg import String  #ข้อพึงสังเกต ตัวแปรที่จะใช้ String จะไม่ใช่ string
#นำเข้าแบบเลือกบางฟังก์ชัน CvBridge() และ CvBridgeError()  
from cv_bridge import CvBridge, CvBridgeError


#----------------------------------------------------------
#------------------------  เริ่มต้น --------------------------
# เชื่อมระบบ ประมวลผลภาพ กับระบบ ROS
bridge = CvBridge()

#กำหนด หัวข้อ(Topic) เพื่อกระจายคำสั่งไปยัง
#โหนด(Node)rosrun computer_vision_pkg motor_controller
command_pub = rospy.Publisher("motor_commands",String)


# Function ตรวจสอบว่าค่าที่อ่านได้ เมื่อเทียบกับค่า อ้างอิง ค่าความสว่างของแสงสีเหลือง 100

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
	
	#กำหนดเส้นตรงเป็นการวางแนวของเซ็นเซอร์
	
	gray_image =  cv2.line(gray_image,(min_line+125,500),(max_line-125,500),1,5)
	
	gray_image =  cv2.line(gray_image,(min_line,700),(max_line,700),0,5)
	
	gray_image =  cv2.line(gray_image,(max_line+50,700),(max_line+100,700),1,5)
	gray_image =  cv2.line(gray_image,(min_line-50,700),(min_line-100,700),1,5)
	
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง LLL 
	gray_image = cv2.circle(gray_image,(pos_lll,700),5,255, 1)
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง LL 
	gray_image = cv2.circle(gray_image,(pos_ll,700),5,255, 1)
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง L
	gray_image = cv2.circle(gray_image,(pos_l,700),5,255, 1)
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง C
	gray_image = cv2.circle(gray_image,(pos_c,700),5,255, 1)
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง CC
	gray_image = cv2.circle(gray_image,(pos_cc,500),5,255, 1)
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง R
	gray_image = cv2.circle(gray_image,(pos_r,700),5,255, 1)
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง RR
	gray_image = cv2.circle(gray_image,(pos_rr,700),5,255, 1)
	#กำหนดตำแหน่งเซ็นเซอร์บนเส้นตรง ตำแหน่ง RRR
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

8. Edit file motor_coontroller.cpp use command $nano motor_coontroller.cpp

// Start File

#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

ros::Publisher leftWheelPub, rightWheelPub;

void commandsCallback(const std_msgs::String::ConstPtr& msg) {
  float leftSpeed = 0;
  float rightSpeed = 0;

  std::string command = msg->data;

  if(command == "GO") 
  {
    leftSpeed = -0.7;
    rightSpeed = -0.7;
  }
  else if(command == "GOM") 
  {
    leftSpeed = -0.5;
    rightSpeed = -0.5;
  }
  else if(command == "GO_REALLY_FAST")
  {
    leftSpeed = -1.0; // radians per second
    rightSpeed = -1.0;
  }
  else if(command == "BACK") 
  {
    leftSpeed = 0.5;
    rightSpeed = 0.5;
  } 
  else if(command == "LEFT") 
  {
    leftSpeed = -1.0;
    rightSpeed = -0.5;
  }
  else if(command == "LEFTL") 
  {
    leftSpeed = -0.5;
    rightSpeed = -0.2;
  } 
  else if(command == "RIGHT") 
  {
    leftSpeed = -0.5;
    rightSpeed = -1.0;
  } 
  else if(command == "RIGHTL") 
  {
    leftSpeed = -0.2;
    rightSpeed = -0.5;
  } 
  else 
  {
    leftSpeed = 0.0;  // Stop the robot
    rightSpeed = 0.0;
  }

  // send messages
  std_msgs::Float64 msgLeft, msgRight;
  msgLeft.data = leftSpeed;
  msgRight.data = rightSpeed;
  leftWheelPub.publish(msgLeft);
  rightWheelPub.publish(msgRight);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "motorController");
  ros::NodeHandle n;

  ros::Subscriber commandSub = n.subscribe("motor_commands", 1000, commandsCallback);

  leftWheelPub = n.advertise<std_msgs::Float64>("/left_wheel_controller/command", 1000);
  rightWheelPub = n.advertise<std_msgs::Float64>("/right_wheel_controller/command", 1000);

  ros::spin();

  return 0;
}



9. recompile code with
    
cd ~/catkin_ws
catkin build computer_vision_pkg
source ~/catkin_ws/devel/setup.bash

11. run test on gazebo 
Open new terminal #1

roslaunch computer_vision_pkg gazebo.launch

12. run code controller
    
Open new terminal #2
cd ~/catkin_ws
source devel/setup.bash
rosrun computer_vision_pkg motor_controller
source ~/bashrc
source devel/setup.bash

14. run code planner

Open new terminal #3

source ~/bashrc
source devel/setup.bash
python3 planner.py


  
