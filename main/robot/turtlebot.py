import os,sys,rospy,time,math,cv2,PID,scipy.stats,copy
import numpy as np
import matplotlib.pyplot as plt
from path import*
from apscheduler.schedulers.background import BackgroundScheduler
from geometry_msgs.msg import Pose,Twist,PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates,ContactsState
from sensor_msgs.msg import LaserScan,Image,CompressedImage,Imu,PointCloud2,JointState
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from quaternion_euler import quaternion_to_euler,euler_to_quaternion
from Ros import gazebo_set_model_state
from quintic_polynomials_planner import quintic_polynomials_planner
from PID import PositionalPID
from mpl_toolkits.mplot3d import Axes3D
#import open3d as o3d
np.set_printoptions(threshold=np.inf)

class Turtlebot:
    def __init__(self):
        rospy.init_node('turtlebot', anonymous=True)
        self.vel_msg = Twist()
        self.modelstates=ModelStates()
        self.depth_raw = Image()
        self.rgb_raw = Image()
        self.rgb_compressed = CompressedImage()
        self.imu = Imu()
        #self.world_imu = Imu()
        self.joint_states=JointState()
        self.pointcloud = PointCloud2()
        self.bridge = CvBridge()
        self.odom = Odometry()
        self.ekf_pose = PoseWithCovarianceStamped()
        self.delay=True

        self.sub_scan = rospy.Subscriber('/scan',LaserScan,self.laser_scan_callback)
        self.sub_depth_raw = rospy.Subscriber('/camera/depth/image_raw',Image,self.depth_raw_callback)
        self.sub_pointcloud = rospy.Subscriber('/camera/depth/points', PointCloud2,callback=self.pointcloud_callback)
        self.sub_rgb_compressed = rospy.Subscriber('/camera/rgb/image_raw/compressed',CompressedImage,self.rgb_compressed_callback)
        self.sub_rgb_raw = rospy.Subscriber('/camera/rgb/image_raw',Image,self.rgb_raw_callback)
        self.sub_model_states = rospy.Subscriber('/gazebo/model_states',ModelStates,self.model_states_callback)
        self.sub_contact = rospy.Subscriber('/turtlebot3_contact_sensor', ContactsState, self.contact_callback)
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.sub_joint_state = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.sub_ekf_pose = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.ekf_pose_callback)
        self.pub_vel = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
        #self.pub_world_acc = rospy.Publisher('/world_imu',Imu,queue_size = 1)
        self.rate = rospy.Rate(1000)

        self.imu_list=[]
        self.position=[0,0,0]
        self.angle=[0,0,0]
        self.laser = None
        self.collision=False
        self.last_joint_states={'wheel_right_joint':0,'wheel_left_joint':0}
        self.pose_list=[]
        self.ekf_pose_list=[]

        self.imu_frequency=10
        #self.imu_period=1/self.imu_frequency
        self.imu_schedule = BackgroundScheduler()
        self.imu_schedule.add_job(self.imu_collector, 'interval', seconds=0.1)
        self.imu_schedule.start(paused=True)

        self.max_distance=10
        self.Gaussian_speed_min=0.15
        self.Gaussian_speed_max=0.35
        self.maxspeed_min=0.3
        self.maxspeed_max=0.7

        self.P=1
        self.I=0.0 #0.0001
        self.D=0.00000 #0.00005
        self.PositionalPidx = PositionalPID(self.P,self.I,self.D)
        self.PositionalPidz = PositionalPID(self.P,self.I,self.D)

        time.sleep(1)
        self.delay=False
        #self.reset_joint_states()
        time.sleep(1)

    def imu_collector(self):
        data=self.get_world_imu()
        self.imu_list.append(data)

    def save_imu(self,name):
        np.save(name,np.array(self.imu_list))

    def reset_joint_states(self):
        self.last_joint_states['wheel_right_joint']=self.joint_states.position[0]
        self.last_joint_states['wheel_left_joint']=self.joint_states.position[1]

    def forward_detection(self):
        left=self.laser[:15]
        right=self.laser[344:]
        for i in left:
            if i<1:
                return False
        for i in right:
            if i<1:
                return False
        return True

    def odom_callback(self,data):
        self.odom=data

    def ekf_pose_callback(self,data):
        self.ekf_pose = data

    def contact_callback(self,data):
        if data.states==[]:
            self.collision=False
        else:
            self.collision=True

    def laser_scan_callback(self,data):
        self.laser = data.ranges

    def depth_raw_callback(self,data):
        self.depth_raw = data

    def pointcloud_callback(self,data):
        self.pointcloud = data

    def rgb_raw_callback(self,data):
        self.rgb_raw= data

    def rgb_compressed_callback(self,data):
        self.rgb_compressed= data

    def imu_callback(self,data):
        self.imu=data

    def joint_state_callback(self,data):
        self.joint_states=data

    def model_states_callback(self,data):
        index=0
        for i in range(len(data.name)):
            index=i
            if data.name[i] =='turtlebot3_waffle':
                break

        if self.delay:
            return

        self.modelstates.name = data.name[index]
        self.modelstates.pose = data.pose[index]
        self.modelstates.twist = data.twist[index]
        self.position[0] = self.modelstates.pose.position.x
        self.position[1] = self.modelstates.pose.position.y
        self.position[2] = self.modelstates.pose.position.z
        angle=quaternion_to_euler(self.modelstates.pose.orientation.x,self.modelstates.pose.orientation.y,
        self.modelstates.pose.orientation.z,self.modelstates.pose.orientation.w)
        self.angle[0] = angle['roll']
        self.angle[1] = angle['pitch']
        if angle['yaw']<0:
            self.angle[2] = angle['yaw']+6.2831
        else:
            self.angle[2] = angle['yaw']

    def set_speed(self,x,z):
        self.vel_msg.linear.x = x
        self.vel_msg.angular.z = z
        self.pub_vel.publish(self.vel_msg)
        self.rate.sleep()

    def set_model_state(self,x,y,z,yaw):
        tf=euler_to_quaternion(0,0,yaw)
        gazebo_set_model_state.main('turtlebot3_waffle',poi_x=x,poi_y=y,poi_z=z,ori_x=tf['x'],ori_y=tf['y'],ori_z=tf['z'],ori_w=tf['w'])
        time.sleep(1)

    def get_collision(self):
        return self.collision

    def get_position(self):
        return {'x':round(self.position[0],4),
                'y':round(self.position[1],4),
                'z':round(self.position[2],4),
                'roll':round(self.angle[0],4),
                'pitch':round(self.angle[1],4),
                'yaw':round(self.angle[2],4)}

    def get_imu(self):
        return {'orientation':
                {'x':round(self.imu.orientation.x,4),
                'y':round(self.imu.orientation.y,4),
                'z':round(self.imu.orientation.z,4),
                'w':round(self.imu.orientation.w,4),},
                'angular_velocity':
                {'x':round(self.imu.angular_velocity.x,4),
                'y':round(self.imu.angular_velocity.y,4),
                'z':round(self.imu.angular_velocity.z,4)},
                'linear_acceleration':
                {'x':round(self.imu.linear_acceleration.x,4),
                'y':round(self.imu.linear_acceleration.y,4),
                'z':round(self.imu.linear_acceleration.z,4)},
                'position':self.get_position()}

    def get_world_imu(self):
        acc=self.get_world_acc()
        return {'orientation':
                {'x':round(self.imu.orientation.x,4),
                'y':round(self.imu.orientation.y,4),
                'z':round(self.imu.orientation.z,4),
                'w':round(self.imu.orientation.w,4),},
                'angular_velocity':
                {'x':round(self.imu.angular_velocity.x,4),
                'y':round(self.imu.angular_velocity.y,4),
                'z':round(self.imu.angular_velocity.z,4)},
                'linear_acceleration':
                {'x':round(acc['x'],4),
                'y':round(acc['y'],4),
                'z':round(self.imu.linear_acceleration.z,4)},
                'position':self.get_position()}

    def get_imu_list(self):
        return self.imu_list

    def get_pointcloud(self):
        points = point_cloud2.read_points_list(self.pointcloud, skip_nans=False,field_names = ("x", "y", "z"))
        points = np.array(points)
        return np.around(points.astype(np.float16),decimals=2)

    def get_rgb_raw(self):
        return self.rgb_raw

    def get_rgb_raw_cv(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.rgb_raw, desired_encoding='bgr8')
        return cv_image

    def get_rgb_compressed(self):
        return self.rgb_compressed

    def get_rgb_compressed_cv(self):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(self.rgb_compressed, desired_encoding='bgr8')
        return cv_image

    def get_depth_raw(self):
        return self.depth_raw

    def get_depth_raw_cv(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.depth_raw, desired_encoding='passthrough')
        return cv_image

    def get_joint_states_difference(self):
        right=abs(self.joint_states.position[0]-self.last_joint_states['wheel_right_joint'])
        left=abs(self.joint_states.position[1]-self.last_joint_states['wheel_left_joint'])
        all=right+left
        self.reset_joint_states()
        return {'wheel_right':right,'wheel_left':left,'all':all}

    def forward(self):
        self.set_speed(0.05,0)
        time.sleep(1)
        self.stop_bot()

    def turn_left(self):
        self.set_speed(0,0.1)
        time.sleep(1)
        self.stop_bot()

    def turn_right(self):
        self.set_speed(0,-0.1)
        time.sleep(1)
        self.stop_bot()

    def depth_image_np(self):
        image=self.get_depth_raw_cv()
        array=np.asarray(image.copy())
        array=np.nan_to_num(array)
        if np.max(array)!=0:
            array/=np.max(array)
        else:
            array[:]=0.1
        return array.flatten()

    def depth_sum(self):
        image=self.get_depth_raw_cv()
        array=np.asarray(image.copy())
        array=np.nan_to_num(array)
        return array.sum()

    def check_rgb_image(self):
        cv2.namedWindow("rgb_image",cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("rgb_image", 500, 500)
        image=bot.get_rgb_raw_cv()
        cv2.imshow("rgb_image",image)
        cv2.waitKey()

    def check_depth_image(self):
        cv2.namedWindow("depth_image",cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("depth_image", 500, 500)
        image=bot.get_depth_raw_cv()
        cv2.imshow("depth_image",image)
        cv2.waitKey()

    def stop_bot(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.pub_vel.publish(self.vel_msg)
        self.rate.sleep()

    def get_radian(self,current,target):
        radian=0.0
        if (target[1]-current[1]) >=0 and (target[0]-current[0])<=0:
            radian = 3.1415 - round(math.atan(abs(target[1]-current[1])/abs(target[0]-current[0])), 4)
        elif (target[1]-current[1]) <=0 and (target[0]-current[0])<=0:
            radian =  3.1415 + round(math.atan(abs(target[1]-current[1])/abs(target[0]-current[0])), 4)
        elif (target[1]-current[1]) <=0 and (target[0]-current[0])>=0:
            radian = 6.2831 - round(math.atan(abs(target[1]-current[1])/abs(target[0]-current[0])), 4)
        else:
            radian=round(math.atan(abs(target[1]-current[1])/abs(target[0]-current[0])), 4)
        return radian

    def twist(self,radius,method):
        speed_offset=0.01
        speed_max=0.6
        normalize_z=abs(radius-self.angle[2])
        imu_temp = copy.deepcopy(self.imu)
        while abs(radius-self.angle[2])>0.01 and not rospy.is_shutdown():
            if self.get_collision():
                print('-----------------')
                print('collision!')
                print('-----------------')
                self.stop_bot()
                return False
            self.vel_msg.linear.x = 0
            if method =='PID':
                self.PositionalPidz.StepSignal = radius
                if(self.PositionalPidz.Get_Singal(self.angle[2]))>0:
                    self.vel_msg.angular.z = speed_offset + self.PositionalPidz.Get_Singal(self.angle[2])
                else:
                    self.vel_msg.angular.z = -speed_offset + self.PositionalPidz.Get_Singal(self.angle[2])
            elif method=='Gaussian':
                if(self.PositionalPidz.Get_Singal(self.angle[2]))>0:
                    self.vel_msg.angular.z = speed_offset + scipy.stats.norm.pdf(radius - self.angle[2],loc = normalize_z/2,scale = 0.2)*self.Gaussian_speed
                else:
                    self.vel_msg.angular.z = -speed_offset - scipy.stats.norm.pdf(radius - self.angle[2],loc = normalize_z/2,scale = 0.2)*self.Gaussian_speed
            if abs(self.vel_msg.angular.z)>speed_max:
                self.vel_msg.angular.z = speed_max if self.vel_msg.angular.z>0 else -speed_max
            self.pub_vel.publish(self.vel_msg)
            if imu_temp != self.imu:
                self.imu_list.append(self.get_imu())
                imu_temp = copy.deepcopy(self.imu)
            self.rate.sleep()
        self.stop_bot()
        return True

    def advance(self,x,y,method):
        speed=0.1
        normalize_x=math.sqrt((self.position[0]-x)*(self.position[0]-x)+(self.position[1]-y)*(self.position[1]-y))
        pre_distance = math.sqrt((self.position[0]-x)**2+(self.position[1]-y)**2)+0.5
        Gaussian_speed=pre_distance/self.max_distance*(self.Gaussian_speed_max-self.Gaussian_speed_min)+self.Gaussian_speed_min
        max_speed = pre_distance/self.max_distance*(self.maxspeed_max-self.maxspeed_min)+self.maxspeed_min
        imu_temp = copy.deepcopy(self.imu)
        pose_temp = copy.deepcopy(self.odom)
        ekf_pose_temp = copy.deepcopy(self.ekf_pose)
        while abs(self.position[0]-x)>0.01 or abs(self.position[1]-y)>0.01 and not rospy.is_shutdown():
            if self.get_collision():
                print('-----------------')
                print('collision!')
                print('-----------------')
                self.stop_bot()
                return False
            distance = math.sqrt((self.position[0]-x)**2+(self.position[1]-y)**2)
            if(distance>pre_distance):
                break
            else:
                pre_distance=distance+0.001

            if method=='PID':
                self.PositionalPidx.StepSignal = 0
                self.vel_msg.linear.x = 0.1 + 0.5 * -self.PositionalPidx.Get_Singal(distance)/normalize_x
            elif method=='Gaussian':
                if 0.1 + scipy.stats.norm.pdf((distance-normalize_x/2)/normalize_x,loc = 0,scale = 0.2)*Gaussian_speed >max_speed:
                    self.vel_msg.linear.x = max_speed
                else:
                    self.vel_msg.linear.x = 0.1 + scipy.stats.norm.pdf((distance-normalize_x/2)/normalize_x,loc = 0,scale = 0.2)*Gaussian_speed

            self.vel_msg.angular.z = 0
            if abs(self.get_radian(self.position,[x,y])-self.angle[2])>0.01:
                self.PositionalPidz.StepSignal = self.get_radian(self.position,[x,y])
                if abs(self.get_radian(self.position,[x,y])-self.angle[2])>3.14:
                    self.vel_msg.angular.z = -speed if self.PositionalPidz.Get_Singal(self.angle[2])>0 else speed
                else:
                    self.vel_msg.angular.z = speed if self.PositionalPidz.Get_Singal(self.angle[2])>0 else -speed
            self.pub_vel.publish(self.vel_msg)
            if imu_temp != self.imu:
                self.imu_list.append(self.get_imu())
                imu_temp = copy.deepcopy(self.imu)
            if pose_temp != self.odom:
                self.pose_list.append(self.odom.pose.pose.position)
                pose_temp = copy.deepcopy(self.odom)
            if ekf_pose_temp != self.ekf_pose:
                self.ekf_pose_list.append(self.ekf_pose.pose.pose.position)
                ekf_pose_temp = copy.deepcopy(self.ekf_pose)
            self.rate.sleep()
        self.stop_bot()
        return True

    def move_continue(self,waypoints,speed,yaw):
        speed_offset=0.4
        twist_offset=1
        self.twist(self.get_radian(self.position,waypoints[10]),"PID")
        self.imu_list=[]
        self.imu_schedule.resume()
        for i in range(len(waypoints)):
            point=waypoints[i]
            while math.sqrt((self.position[0]-point[0])**2+(self.position[1]-point[1])**2)>0.1 and not rospy.is_shutdown():

                speed_x=math.sqrt((self.position[0]-point[0])**2+(self.position[1]-point[1])**2)\
                /abs(self.get_radian(self.position,[point[0],point[1]])-self.angle[2])*speed_offset

                if speed_x>speed_offset:
                    speed_x=speed_offset

                self.vel_msg.linear.x = speed_x
                #self.vel_msg.linear.x = speed[i]

                speed_z=self.get_radian(self.position,[point[0],point[1]])-self.angle[2]
                if abs(self.get_radian(self.position,[point[0],point[1]])-self.angle[2])>3.14:
                    if speed_z>0:
                        speed_z=speed_z-6.2831
                    else:
                        speed_z=speed_z+6.2831

                self.vel_msg.angular.z = speed_z
                #self.vel_msg.angular.z = yaw[i]
                #print(self.vel_msg.angular.z)

                #help = self.protect_control()

                self.pub_vel.publish(self.vel_msg)
                self.rate.sleep()
                if self.collision:
                    return False
                #if help:
                    #break
        self.imu_schedule.pause()
        self.stop_bot()
        return True

    def move_to_coordinate(self,x,y):
        normalize_z=abs(self.get_radian(self.position,[x,y])-self.angle[2])
        normalize_x=math.sqrt((self.position[0]-x)*(self.position[0]-x)+(self.position[1]-y)*(self.position[1]-y))
        original_self_angle=self.angle[2]
        original_target_angle=self.get_radian(self.position,[x,y])
        self.imu_list=[]
        #self.imu_schedule.resume()
        bool_twist=self.twist(original_target_angle,'PID')
        bool_advance=self.advance(x,y,'Gaussian')
        #self.imu_schedule.pause()
        if bool_advance and bool_twist:
            return True
        else:
            return False

    def calculate_grid_tf(self,direction):
        x=0
        y=0
        if direction==0:
            x=int(self.position[0])
            y=int(self.position[1])+1
        elif direction==1:
            x=int(self.position[0])+1
            y=int(self.position[1])+1
        elif direction==2:
            x=int(self.position[0])+1
            y=int(self.position[1])
            fix=True
        elif direction==3:
            x=int(self.position[0])+1
            y=int(self.position[1])-1
        elif direction==4:
            x=int(self.position[0])
            y=int(self.position[1])-1
        elif direction==5:
            x=int(self.position[0])-1
            y=int(self.position[1])-1
        elif direction==6:
            x=int(self.position[0])-1
            y=int(self.position[1])
        elif direction==7:
            x=int(self.position[0])-1
            y=int(self.position[1])+1
        return x+0.5,y+0.5

    def move_to_grid(self,direction):
        x,y=self.calculate_grid_tf(direction)
        self.move_to_coordinate(x,y)

    def turn8(self,direction):
        pi=3.14159
        unit=pi/4
        if direction==0:
            self.twist(unit*2,'PID')
        elif direction==1:
            self.twist(unit*1,'PID')
        elif direction==2:
            self.twist(unit*0,'PID')
        elif direction==3:
            self.twist(unit*7,'PID')
        elif direction==4:
            self.twist(unit*6,'PID')
        elif direction==5:
            self.twist(unit*5,'PID')
        elif direction==6:
            self.twist(unit*4,'PID')
        elif direction==7:
            self.twist(unit*3,'PID')

    def forward_grid(self):
        pi=3.14159
        unit=pi/4
        direction=0
        temp=7
        for i in range(8):
            if(temp>abs(i*unit-self.angle[2])):
                temp=abs(i*unit-self.angle[2])
                direction=i
        if direction<3:
            direction = 2-direction
        else:
            direction = 10-direction
        x,y=self.calculate_grid_tf(direction)
        self.imu_list=[]
        self.imu_schedule.resume()
        self.advance(x,y,'Gaussian')
        self.imu_schedule.pause()

    def generate_sensing(self):
        offset=0.2
        environment=np.zeros((5,5))
        environment[2][2]=0.5
        criterion_0=[1.5-offset,2.5-offset]
        criterion_31=[2.9-offset] #59
        criterion_45=[2.1-offset,3.5-offset]
        environment[2][3]=self.laser[0]<=criterion_0[0] if 1 else 0
        environment[2][4]=self.laser[0]<=criterion_0[1] if 1 else 0
        environment[1][4]=self.laser[31]<=criterion_31[0] if 1 else 0
        environment[1][3]=self.laser[45]<=criterion_45[0] if 1 else 0
        environment[0][4]=self.laser[45]<=criterion_45[1] if 1 else 0
        environment[0][3]=self.laser[59]<=criterion_31[0] if 1 else 0
        environment[1][2]=self.laser[90]<=criterion_0[0] if 1 else 0
        environment[0][2]=self.laser[90]<=criterion_0[1] if 1 else 0
        environment[0][1]=self.laser[90+31]<=criterion_31[0] if 1 else 0
        environment[1][1]=self.laser[90+45]<=criterion_45[0] if 1 else 0
        environment[0][0]=self.laser[90+45]<=criterion_45[1] if 1 else 0
        environment[1][0]=self.laser[90+59]<=criterion_31[0] if 1 else 0
        environment[2][1]=self.laser[180]<=criterion_0[0] if 1 else 0
        environment[2][0]=self.laser[180]<=criterion_0[1] if 1 else 0
        environment[3][0]=self.laser[180+31]<=criterion_31[0] if 1 else 0
        environment[3][1]=self.laser[180+45]<=criterion_45[0] if 1 else 0
        environment[4][0]=self.laser[180+45]<=criterion_45[1] if 1 else 0
        environment[4][1]=self.laser[180+59]<=criterion_31[0] if 1 else 0
        environment[3][2]=self.laser[270]<=criterion_0[0] if 1 else 0
        environment[4][2]=self.laser[270]<=criterion_0[1] if 1 else 0
        environment[4][3]=self.laser[270+31]<=criterion_31[0] if 1 else 0
        environment[3][3]=self.laser[270+45]<=criterion_45[0] if 1 else 0
        environment[4][4]=self.laser[270+45]<=criterion_45[1] if 1 else 0
        environment[3][4]=self.laser[270+59]<=criterion_31[0] if 1 else 0
        return environment

    def protect_control(self):
        speed_x=0.1
        speed_z=0.4
        sensing_angle=90
        sensing_range=0.45
        left=self.laser[:sensing_angle]
        right=self.laser[359-sensing_angle:]

        for i in left:
            if i <sensing_range:
                self.vel_msg.angular.z = -speed_z
                self.vel_msg.linear.x = speed_x
                return True

        for i in right:
            if i <sensing_range:
                self.vel_msg.angular.z = speed_z
                self.vel_msg.linear.x = speed_x
                return True

        return False
        #self.pub_vel.publish(self.vel_msg)

    def get_grid_position(self):
        return {'x':int(self.position[0]),'y':int(self.position[1])}

    def get_speed(self):
        degree = self.angle[2]
        speed = self.odom.twist.twist.linear.x
        speed_x=0
        speed_y=0
        angle_point=[math.radians(90),math.radians(180),math.radians(270),math.radians(360)]
        if degree>angle_point[3]:
            degree = degree - angle_point[3]
        if degree<=angle_point[0]:
            speed_x = speed * math.cos(degree)
            speed_y = speed * math.sin(degree)
        elif degree<=angle_point[1]:
            speed_x = -speed * math.sin(degree-angle_point[0])
            speed_y = speed * math.cos(degree-angle_point[0])
        elif degree<=angle_point[2]:
            speed_x = -speed * math.cos(degree-angle_point[1])
            speed_y = -speed * math.sin(degree-angle_point[1])
        elif degree<=angle_point[3]:
            speed_x = speed * math.sin(degree-angle_point[2])
            speed_y = -speed * math.cos(degree-angle_point[2])
        return {'x':speed_x,'y':speed_y}

    def get_world_acc(self):
        degree = self.angle[2]
        acc = self.imu.linear_acceleration.x
        acc_x=0
        acc_y=0
        angle_point=[math.radians(90),math.radians(180),math.radians(270),math.radians(360)]
        if degree>angle_point[3]:
            degree = degree - angle_point[3]
        if degree<=angle_point[0]:
            acc_x = acc * math.cos(degree)
            acc_y = acc * math.sin(degree)
        elif degree<=angle_point[1]:
            acc_x = -acc * math.sin(degree-angle_point[0])
            acc_y = acc * math.cos(degree-angle_point[0])
        elif degree<=angle_point[2]:
            acc_x = -acc * math.cos(degree-angle_point[1])
            acc_y = -acc * math.sin(degree-angle_point[1])
        elif degree<=angle_point[3]:
            acc_x = acc * math.sin(degree-angle_point[2])
            acc_y = -acc * math.cos(degree-angle_point[2])
        return {'x':acc_x,'y':acc_y}

def plot_3D(data):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_zlabel('Z', color='k')
    ax.set_ylabel('Y', color='k')
    ax.set_xlabel('X', color='k')
    #ax.set_xlim(0, 10)
    #ax.set_xticks(np.arange(0,10,1))
    #ax.set_ylim(0, 10)
    #ax.set_yticks(np.arange(0,10,1))
    #ax.set_zlim(0, 10)
    #ax.set_title("title")
    #ax.legend(loc='lower right')
    ax.plot(data[0], data[1], data[2], 'o')
    ax.view_init(40, -130)
    time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    plt.show()


if __name__ == '__main__':
    bot=Turtlebot()
    time.sleep(1)
    '''
    #bot.check_depth_image()
    #bot.check_rgb_image()
    #data = np.array(bot.get_pointcloud())
    #data = np.reshape(data,(10,10))
    #print(data)
    points = bot.get_pointcloud()
    #print(points)
    points_t = np.transpose(points)
    #print(points_t)
    points_t_max = np.ndarray.max(points_t,axis=1)
    points_t_min = np.ndarray.min(points_t,axis=1)
    points_t[0] = points_t[0]-points_t_min[0]
    points_t[1] = points_t[1]-points_t_min[1]
    points_t_max = np.ndarray.max(points_t,axis=1)
    #print(points_t_max)
    points_t[1] = points_t_max[1] - points_t[1]
    #print(points_t)
    points = np.transpose(points_t)

    plot_3D([points_t[2],points_t[0],points_t[1]])
    '''
    depth = bot.check_rgb_image()
    input()
    #rgb = bot.get_rgb_raw_cv()
    #print(depth.shape)
    #print(rgb.shape)
    #bot.check_depth_image()
    #print(depth)
    #print(depth)

    cv2.imwrite('depth.png',depth*20)
    #cv2.imwrite('rgb.png',rgb)
    rgb = o3d.io.read_image('rgb.png')
    depth = o3d.io.read_image('depth.png')
    #print(rgb.dtype)
    #print(depth.dtype)
    #print(np.array(depth))
    #depth = depth*10
    #depth = depth.astype('uint8')
    #rgb = rgb.astype('uint8')
    rgb = o3d.geometry.Image(np.array(np.asarray(rgb)[:, :, :3]))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, depth,depth_scale =1, depth_trunc = 1000, convert_rgb_to_intensity = False)
    cam = o3d.camera.PinholeCameraIntrinsic(640, 480, 402.3, 402.3, 320.5, 240.5)#W, H, Fx, Fy, Cx, Cy

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)
    #depth = o3d.geometry.Image(np.array(depth).astype('uint16'))
    #pcd = o3d.geometry.PointCloud.create_from_depth_image(depth,cam)

    o3d.visualization.draw_geometries([pcd])
