# coding=utf-8
#!/usr/bin/python
# import the necessary packages 
#from xml.etree.ElementTree import tostring
from cv_bridge import CvBridge
import cv2 
import numpy as np
import rospy
from sensor_msgs.msg import Image
import os
from std_msgs.msg import Bool

red1=np.array([0,43,46]) 
red2=np.array([180,180,180]) 
bridge = CvBridge()
def image_callback(Image):
    image=bridge.imgmsg_to_cv2(Image, "bgr8")
    image=image[400:720,300:900]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV) 
    mask=cv2.inRange(hsv,red1,red2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))#定义结构元素的形状和大小
    dst = cv2.erode(mask, kernel)#腐蚀操作
    count=np.sum(dst)/255
    cv2.putText(dst,str(count),(100,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3)
    cv2.imshow("test",dst)
    cv2.waitKey(10)
    print(count)
    if(count>300):
        pub.publish(True)
    else:
        pub.publish(False)

if __name__ == "__main__":
    rospy.init_node("detetcion",anonymous=True)
    pub = rospy.Publisher("leaf_detect",Bool,queue_size=10)
    image_sub=rospy.Subscriber("/sensors/camera/image_color",Image,image_callback,queue_size=10)
    rospy.spin()



# count_list=[]
# filenames=os.listdir("./test")
    # for i in range(len(filenames)):
    #     if(filenames[i].split(".")[1]=="jpg"):
    #         image = cv2.imread("test/"+filenames[i])
    #         image=image[400:720,300:1000]
    #         cv2.imshow("test1", image)
    #         hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV) 
    #         mask=cv2.inRange(hsv,red1,red2)
    #         kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))#定义结构元素的形状和大小
    #         dst = cv2.erode(mask, kernel)#腐蚀操作
    #         #mask_bin=cv2.threshold(mask,100,255,cv2.THRESH_BINARY)
    #         count=np.sum(dst)/255
    #         count_list.append(count)
    #         if(count>500):
    #             print("启动上桩:%d",count)
    #         cv2.putText(dst,str(count),(100,100),cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3)
    #         cv2.imshow("test", dst) 
    #         print(filenames[i])
    #         cv2.waitKey(0) 


