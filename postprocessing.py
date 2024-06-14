import rclpy
from rclpy.qos import QoSProfile, qos_profile_system_default, qos_profile_sensor_data, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import sys
import os as os
import ezdxf # Import DXF Library
import numpy as np

from matplotlib import path

from std_msgs.msg import Bool, UInt16, UInt64, Int16, Float32, Float64, Float64MultiArray


class PostProcessNode(Node):
    def __init__(self):
        super().__init__('post_process_node_VW')
        self.variable()
        self.rostopic()
        self.create_timer(1, self.ros_pub)

    def variable(self):
        self.Num_VW = 0.0
        self.VW_Draw_Count = []
        self.Draw_X = []
        self.Draw_Y = []
        self.joint_3d = []
        self.pre_status = 0
        self.post_status = 0

    def rostopic(self):
        qos_profile = QoSProfile(
            depth = 1
        )

        # Create subscription
        self.create_subscription(Float64MultiArray, '/Kinematic/Joint_3D', self.callback_joint_3d, 10)
        self.create_subscription(Int16, '/Map/Preprocess_Status_VW', self.pre_status_callback, 100)

        # Create publishers
        self.drawnum_publisher = self.create_publisher(Float64, '/VW/DrawNum', 10)
        self.drawcnt_publisher = self.create_publisher(Float64MultiArray, '/VW/DrawCnt', 10)
        self.draw_x_publisher = self.create_publisher(Float64MultiArray, '/VW/Draw_X', 10)
        self.draw_y_publisher = self.create_publisher(Float64MultiArray, '/VW/Draw_Y', 10)
    
    def callback_joint_3d(self, msg):
        self.joint_3d = msg.data

    def pre_status_callback(self, msg):
        self.pre_status = msg.data   

        if self.pre_status == 0:                                  
            self.post_status = 0
            self.DrawNum = 0.0
            self.DrawCnt = []
            self.Draw_X = []
            self.Draw_Y = []
            
        elif self.pre_status == 1: 
            self.post_status = 1
            self.DrawNum = load_data_from_txt('/usr/bin/download_12agp/virtualwall_working/DrawNum.txt')
            self.DrawCnt = load_data_from_txt('/usr/bin/download_12agp/virtualwall_working/DrawCnt.txt')
            self.Draw_X = load_data_from_txt('/usr/bin/download_12agp/virtualwall_working/DrawX.txt')
            self.Draw_Y = load_data_from_txt('/usr/bin/download_12agp/virtualwall_working/DrawY.txt')
            
        else :
            self.post_status = 0
            # print("pre_status_error")
    	
    def ros_pub(self):

        self.processing_node()

        float_msg = Float64()
        float_msg.data = float(self.VW_Draw_Num_Final)
        self.drawnum_publisher.publish(float_msg)
        
        float_multi_msg = Float64MultiArray()
        for i in range(len(self.VW_Draw_Count_Final)):
            float_multi_msg.data.append(self.VW_Draw_Count_Final[i])
        self.drawcnt_publisher.publish(float_multi_msg)

        float_multi_msg = Float64MultiArray()
        for i in range(len(self.VW_Draw_X_Final)):
            float_multi_msg.data.append(self.VW_Draw_X_Final[i])
        self.draw_x_publisher.publish(float_multi_msg)

        float_multi_msg = Float64MultiArray()
        for i in range(len(self.VW_Draw_Y_Final)):
            float_multi_msg.data.append(self.VW_Draw_Y_Final[i])
        self.draw_y_publisher.publish(float_multi_msg)

    def processing_node(self):

        if self.post_status == 1:
            CoR_X = self.joint_3d[48]
            CoR_Y = self.joint_3d[49]

            Zone_Search_X = CoR_X + np.array([-15, 15, 15, -15, -15])
            Zone_Search_Y = CoR_Y + np.array([-15, -15, 15, 15, -15])
        
            p = path.Path([(Zone_Search_X[0], Zone_Search_Y[0]),(Zone_Search_X[1], Zone_Search_Y[1]),(Zone_Search_X[2], Zone_Search_Y[2]),(Zone_Search_X[3], Zone_Search_Y[3])]);
       
            DrawCnt = []
            for i in range(len(self.DrawCnt)):
                DrawCnt.append(int((self.DrawCnt[i])))

            Draw_X = []
            for i in range(len(self.Draw_X)):
                Draw_X.append(float(self.Draw_X[i]))

            Draw_Y = []
            for i in range(len(self.Draw_Y)):
                Draw_Y.append(float(self.Draw_Y[i]))

            #print(DrawCnt)
            #print(Draw_X)
            #print(Draw_Y)

            # VW_Count = np.array(self.DrawCnt,np.int32)
            VW_Count = DrawCnt
    
            a = 0
            Contact = 0
            aa = 0

            VW_Draw_Num = 0
            Num_VW_Point = 999 * np.ones(shape = (128,))
            VW_Index = 999 * np.ones(shape = (128,))


            for i in range(0,int(self.DrawNum)):

                VW_X_temp = np.zeros(shape = (5,))
                VW_Y_temp = np.zeros(shape = (5,))

                #print(i)
                #print(VW_X_temp)
            
                in_temp = 0

                VW_X_temp[0:(VW_Count[i]+1)] = np.array(list(Draw_X[(sum(VW_Count[0:i])):sum(VW_Count[0:i+1])]) + list([Draw_X[sum(VW_Count[0:i])]]))
                VW_Y_temp[0:(VW_Count[i]+1)] = np.array(list(Draw_Y[(sum(VW_Count[0:i])):sum(VW_Count[0:i+1])]) + list([Draw_Y[sum(VW_Count[0:i])]]))

                VW_X_temp = np.delete(VW_X_temp,np.where((VW_X_temp == 0)))
                VW_Y_temp = np.delete(VW_Y_temp,np.where((VW_Y_temp == 0)))

                for j in range(0,len(VW_X_temp)):
                    a = p.contains_points([(VW_X_temp[j], VW_Y_temp[j])])
                    if a == True:
                        in_temp = 1

                    for k in range(0,4):
                        for l in range(0,VW_Count[i]):
                            if in_temp == 1:
                                Contact = 1
                                break

                            x1 = Zone_Search_X[k]
                            x2 = Zone_Search_X[k+1]
                            y1 = Zone_Search_Y[k]
                            y2 = Zone_Search_Y[k+1]

                            x3 = VW_X_temp[l]
                            x4 = VW_X_temp[l+1]
                            y3 = VW_Y_temp[l]
                            y4 = VW_Y_temp[l+1]

                            Det_T = ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
                            Det_U = ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))

                            if Det_T == 0 or Det_U == 0:
                                dist_13 = sqrt((x1-x3)*(x1-x3) + (y1-y3)*(y1-y3))
                                dist_14 = sqrt((x1-x4)*(x1-x4) + (y1-y4)*(y1-y4))
                                dist_23 = sqrt((x2-x3)*(x2-x3) + (y2-y3)*(y2-y3))
                                dist_24 = sqrt((x2-x4)*(x2-x4) + (y2-y4)*(y2-y4))

                                if dist_13 <= 15 or dist_14 <= 15 or dist_23 <= 15 or dist_24 <= 15:
                                    Contact = 1
                                    break
                                else: 
                                    Contact = 0

                            else:
                                Be_T = ((x1-x3)*(y1-y2)-(y1-y3)*(x1-x2))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
                                Be_U = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
                                
                                if Be_T >= 0 and Be_T <= 1 and Be_U >=0 and Be_U <= 1:
                                    Contact = 1
                                    break
                                else:
                                Contact = 0

                    if Contact == 1:
                        Num_VW_Point[aa] = VW_Count[i]
                        VW_Index[aa] = i
                        aa = aa+1
                        break

            VW_Draw_Num_Final = aa
            VW_Draw_Count_Final = Num_VW_Point
            VW_Index = np.delete(VW_Index,np.where((VW_Index == 999)))
            VW_Draw_Count_Final = np.delete(VW_Draw_Count_Final,np.where(VW_Draw_Count_Final == 999))
            VW_Draw_Count_Final = np.array(VW_Draw_Count_Final,np.int32)
            VW_Index = np.array(VW_Index,np.int32)

            VW_Draw_X_Final = np.zeros(shape = (128,))
            VW_Draw_Y_Final = np.zeros(shape = (128,))

            for bb in range(0,VW_Draw_Num_Final):
                VW_Draw_X_Final[sum(VW_Draw_Count_Final[0:bb]):sum(VW_Draw_Count_Final[0:bb])+VW_Draw_Count_Final[bb]] = Draw_X[sum(VW_Count[0:VW_Index[bb]]):sum(VW_Count[0:VW_Index[bb]])+VW_Count[VW_Index[bb]]]
                VW_Draw_Y_Final[sum(VW_Draw_Count_Final[0:bb]):sum(VW_Draw_Count_Final[0:bb])+VW_Draw_Count_Final[bb]] = Draw_Y[sum(VW_Count[0:VW_Index[bb]]):sum(VW_Count[0:VW_Index[bb]])+VW_Count[VW_Index[bb]]]

            self.VW_Draw_Num_Final = VW_Draw_Num_Final
            self.VW_Draw_Count_Final = VW_Draw_Count_Final
            self.VW_Draw_X_Final = VW_Draw_X_Final
            self.VW_Draw_Y_Final = VW_Draw_Y_Final

        else:
            self.VW_Draw_Num_Final = 0
            self.VW_Draw_Count_Final = []
            self.VW_Draw_X_Final = []
            self.VW_Draw_Y_Final = []


def load_data_from_txt(file_path):
    try:
        # Open the file in read mode
        with open(file_path, 'r') as file:
            # Read lines from the file
            lines = file.readlines()

            # Assuming each line contains space-separated values, split and convert to float
            data = [[float(value) for value in line.split()] for line in lines]

            # Convert the nested list to a NumPy array
            data_array = np.array(data)
            # print(data_array[0:10])

            return data_array
    except Exception as e:
        print(f"Error loading data from {file_path}: {e}")
        return None
            
    
     

def main(args=None):
    rclpy.init(args=args)
    post_process_node_VW = PostProcessNode()
    rclpy.spin(post_process_node_VW)
    post_process_node_VW.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
