#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.
by Takuya Yamaguchi @dashimaki360
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import sys

from geometry_msgs.msg import Twist


class AllSensorBot(object):
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # image publisher 追加した
        self.image_pub = rospy.Publisher('processed_image', Image, queue_size=10)


        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
#            self.img = None # うまくいかなかった
            cols = 640
            rows = 480
            self.img = np.full((rows, cols, 3), 0, dtype=np.uint8)
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # update twist
            twist = Twist()
            twist.linear.x = 0.1; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            # publish twist topic
            self.vel_pub.publish(twist)
            r.sleep()


    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        """ # dockerの人はコメントアウト
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)
        """

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('aimTarget')
    bot = AllSensorBot(use_lidar=False, use_camera=True, use_imu=False,
                       use_odom=False, use_joint_states=False)
#    bot.strategy()

    # velocity publisher
    vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

    count = 0
    count_th1=2
    count_th2=1
    mask_sum = 0
#    judge_th = 1000000
    judge_th = 10000

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        # AllSensorBotクラス の imageCallback関数で取得された画像データを取得
        bgr_image = bot.img

        # HSV色空間に変換
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # 画像の二値化のための範囲指定。HSVで。
#       lower = np.array([-30, 100, 50]) # red
#       upper = np.array([30, 255, 255]) # red
#        lower = np.array([30, 100, 50]) # yellow
#        upper = np.array([50, 255, 255]) # yellow
       lower = np.array([120, 100, 50]) # blue
       upper = np.array([140, 255, 255]) # blue
#       lower = np.array([40, 100, 50]) # green
#       upper = np.array([60, 255, 255]) # green

        # 値が指定した範囲内の画素は255、範囲外の画素を0にする二値化
        mask_image = cv2.inRange(hsv_image, lower, upper)
	mask_sum = mask_image.sum()
        print("合計値：　%d" % mask_sum)

        # 先程二値化した画像をマスク画像としてBGR画像を切り抜き
        processed_image = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_image)

        # 重心を求める
        mom = cv2.moments(mask_image)
        cx, cy = 0, 0
        if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
            cx = int(mom["m10"]/mom["m00"])
            cy = int(mom["m01"]/mom["m00"])
        print(cx, cy)

        # 求めた重心の位置を示すために紫色の点を描画
        color = (255, 0, 255)
        processed_image = cv2.circle(processed_image, (cx, cy), 3, color, -1)

        # 加工した画像をROS Topicの形式に変換してpublish
        image_msg = bot.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        bot.image_pub.publish(image_msg)


        # 真ん中を向くように方向転換
        diff_pix = cx - 320
        anglular_z = 0

        if abs(diff_pix) < 320 and diff_pix > 20:
            anglular_z = -0.3
        elif abs(diff_pix) < 320 and diff_pix < -20:
            anglular_z = 0.3
        else:
            anglular_z = 0.0
            
        print(anglular_z)

        actionMode = rospy.get_param("actionMode")
        print("actionMode = %s" %(actionMode))

        #3/2 shen
        if cx != 0:
        #if mask_sum > judge_th:
            count+=1
        else:
            if actionMode == "aimEnemy":
                count-=1
            else:
                count=0

        if count > count_th1:
            #敵を発見した場合
            count = count_th1
            if actionMode != "aimEnemy":
                rospy.set_param('actionMode','aimEnemy')
        else:
      	    if actionMode == "aimEnemy":
                #敵を見失った場合
                if count < count_th2:
                    rospy.set_param('actionMode','patrol')
            #else:
                #敵が見当たらない場合
                #２つ以上の動作モードの可能性もあるため、モード設定を変更しないほうがよい
                #if(actionMode!="patrol"):
                #    rospy.set_param('actionMode','patrol')

        r.sleep()
        #2/27 shen
        actionMode = rospy.get_param("actionMode")
        print("actionMode = %s" %(actionMode))

        #敵への追従性を向上するために、常にトピックをPubする
        if actionMode== "aimEnemy":
        #ROSparam設定してから反映されるまでに時間かかるため、敵への追従をよくするためにmask_sumを使って判定する
        #if mask_sum > judge_th:
            twist = Twist()
            #twist.linear.x = 0.2; twist.linear.y = 0; twist.linear.z = 0
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = anglular_z
            vel_pub.publish(twist)

        r.sleep()
