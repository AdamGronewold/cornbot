#make the ros node class available for inheritance by our custom node
import rclpy
import serial
import time
from pynmeagps import NMEAReader


from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import uuid

# Writing to an excel 
# sheet using Python
import xlwt
from xlwt import Workbook
from datetime import datetime, timezone


class DAQNode(Node): #Make a subclass of the Node class called "DAQode"

    def __init__(self):
        super().__init__('daq_node') #call the constructor of the class from which this class is inhereted, providing the new node name as "gnss_node"

        self.logging=False

        self.current_row_sheet1=0
        self.current_row_sheet2=0
        self.current_row_sheet3=0
        
        self.unique_filename = str(uuid.uuid4()) 
        self.log_data_subscriber = self.create_subscription(Bool, 'cornbot/data_logging', self.logging_sub_callback, 1) #subscriber which waits for message from the data_logging topic "True" or "False" to notify the daq node to record data from subscribers
        
        self.subscriber1=self.create_subscription(Vector3, 'cornbot/IMU/euler_angles_topic', self.write_euler_angles_callback, 1)
        self.subscriber2=self.create_subscription(Vector3, 'cornbot/IMU/accelerometer_topic', self.write_accelerometer_callback, 1)
        self.subscriber3=self.create_subscription(Vector3, 'cornbot/IMU/linear_accel_topic', self.write_lin_accelerometer_callback, 1)
        self.subscriber4=self.create_subscription(Vector3, 'cornbot/IMU/grav_accel_topic', self.write_grav_accelerometer_callback, 1)
        self.subscriber5=self.create_subscription(Vector3, 'cornbot/feeler_angles_topic', self.write_feeler_callback, 1)
        
        self.subscriber6=self.create_subscription(String, 'cornbot/gnss_nmea_string_stream_topic', self.write_gnss_string_callback, 1)
        self.subscriber7=self.create_subscription(String, 'cornbot/gnss_nmea_raw_stream_topic', self.write_gnss_raw_callback, 1)
        self.subscriber8=self.create_subscription(String, 'cornbot/gnss_nmea_parsed_stream_topic', self.write_gnss_parsed_callback, 1)
        
        self.subscriber9=self.create_subscription(String, 'cornbot/speed_ref_topic', self.write_ref_speed_callback, 1)
        self.subscriber10=self.create_subscription(Float32, 'cornbot/wheel_rpm_left', self.write_wheel_speed_left_callback, 1)
        self.subscriber11=self.create_subscription(Float32, 'cornbot/wheel_rpm_right', self.write_wheel_speed_right_callback, 1)
        
        self.get_logger().info('DAQ node started!')
       
    def write_ref_speed_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet2.write(self.current_row_sheet2, 1, str(datetime.now(timezone.utc)))
            self.sheet2.write(self.current_row_sheet2, 3, msg.data)   
            
            #self.wb.save(self.current_log_file)
            
            self.current_row_sheet2+=1
            
        if self.logging==False:
            pass
            
    def write_wheel_speed_left_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet2.write(self.current_row_sheet2, 1, str(datetime.now(timezone.utc)))
            self.sheet2.write(self.current_row_sheet2, 5, msg.data)  
            
            self.wb.save(self.current_log_file)
            
            self.current_row_sheet2+=1 
            
        if self.logging==False:
            pass

    def write_wheel_speed_right_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet2.write(self.current_row_sheet2, 1, str(datetime.now(timezone.utc)))
            self.sheet2.write(self.current_row_sheet2, 4, msg.data)  
            
            #self.wb.save(self.current_log_file)
            
            self.current_row_sheet2+=1
             
        if self.logging==False:
            pass
            
    def write_gnss_string_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet3.write(self.current_row_sheet3, 1, str(datetime.now(timezone.utc)))
            self.sheet3.write(self.current_row_sheet3, 3, msg.data)  
            
            self.wb.save(self.current_log_file)
            
            self.current_row_sheet3+=1
             
        if self.logging==False:
            pass
            
    def write_gnss_raw_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet3.write(self.current_row_sheet3, 1, str(datetime.now(timezone.utc)))
            self.sheet3.write(self.current_row_sheet3, 4, msg.data)  
            
            self.wb.save(self.current_log_file)
            
            self.current_row_sheet3+=1
             
        if self.logging==False:
            pass
            
    def write_gnss_parsed_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet3.write(self.current_row_sheet3, 1, str(datetime.now(timezone.utc)))
            self.sheet3.write(self.current_row_sheet3, 5, msg.data)  
            
            self.wb.save(self.current_log_file)
            
            self.current_row_sheet3+=1 
            
        if self.logging==False:
            pass
            
    def write_euler_angles_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet1.write(self.current_row_sheet1, 1, str(datetime.now(timezone.utc)))
            self.sheet1.write(self.current_row_sheet1, 6, msg.x)  
            self.sheet1.write(self.current_row_sheet1, 7, msg.y)  
            self.sheet1.write(self.current_row_sheet1, 8, msg.z)              
            
            #self.wb.save(self.current_log_file)
            
            self.current_row_sheet1+=1
             
        if self.logging==False:
            pass

    def write_accelerometer_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet1.write(self.current_row_sheet1, 1, str(datetime.now(timezone.utc)))
            self.sheet1.write(self.current_row_sheet1, 10, msg.x)  
            self.sheet1.write(self.current_row_sheet1, 11, msg.y)  
            self.sheet1.write(self.current_row_sheet1, 12, msg.z)
            
            self.wb.save(self.current_log_file)
                          
            self.current_row_sheet1+=1 
            
        if self.logging==False:
            pass

    def write_lin_accelerometer_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet1.write(self.current_row_sheet1, 1, str(datetime.now(timezone.utc)))
            self.sheet1.write(self.current_row_sheet1, 14, msg.x)  
            self.sheet1.write(self.current_row_sheet1, 15, msg.y)  
            self.sheet1.write(self.current_row_sheet1, 16, msg.z) 
            
            self.wb.save(self.current_log_file)
                         
            self.current_row_sheet1+=1
             
        if self.logging==False:
            pass

    def write_grav_accelerometer_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet1.write(self.current_row_sheet1, 1, str(datetime.now(timezone.utc)))
            self.sheet1.write(self.current_row_sheet1, 18, msg.x)  
            self.sheet1.write(self.current_row_sheet1, 19, msg.y)  
            self.sheet1.write(self.current_row_sheet1, 20, msg.z)              
            
            self.wb.save(self.current_log_file)
            
            self.current_row_sheet1+=1 
        if self.logging==False:
            pass

    def write_feeler_callback(self, msg):
        if self.logging==True:
            self.get_logger().info("Data logging in process.")
            self.sheet1.write(self.current_row_sheet1, 1, str(datetime.now(timezone.utc)))
            self.sheet1.write(self.current_row_sheet1, 3, msg.x)  
            self.sheet1.write(self.current_row_sheet1, 4, msg.y)  
            
            self.wb.save(self.current_log_file)
             
            self.current_row_sheet1+=1 
            
        if self.logging==False:
            pass
            
    def logging_sub_callback(self, msg):
    	if msg.data==True:
    	    self.unique_filename = str(uuid.uuid4()) 
    	    self.get_logger().info("Beginning data logging under /cornbot/log/%s" %self.unique_filename)
    	    self.logging=True
    	    self.current_log_file='/home/adamgronewold/ros2_ws/src/cornbot/log/' + self.unique_filename + '.xls'
    	    #self.f=open(current_log_file, 'a')
    	    
    	    # Workbook is created
    	    self.wb = Workbook()
    	    # add_sheet is used to create sheet.
    	    self.sheet1 = self.wb.add_sheet('Sensor Data')
    	    self.sheet2 = self.wb.add_sheet('Motor Data')
    	    self.sheet3 = self.wb.add_sheet('GPS Data')
    	    self.sheet1.write(0, 0, 'SENSOR DATA STREAM FROM CORNROBOT')
    	    self.sheet2.write(0, 0, 'MOTOR DATA STREAM FROM CORNROBOT')
    	    self.sheet3.write(0, 0, 'GPS DATA STREAM FROM CORNROBOT')

    	    self.sheet1.write(1, 0, 'START TIME')
    	    self.sheet1.write(1, 1, str(datetime.now(timezone.utc)))
    	    self.sheet2.write(1, 0, 'START TIME')
    	    self.sheet2.write(1, 1, str(datetime.now(timezone.utc)))
    	    self.sheet3.write(1, 0, 'START TIME')
    	    self.sheet3.write(1, 1, str(datetime.now(timezone.utc)))
    	    
    	    self.sheet1.write(3, 1, 'TIME')
    	    self.sheet2.write(3, 1, 'TIME')
    	    self.sheet3.write(3, 1, 'TIME')
    	    
    	    self.sheet2.write(3, 3, 'REFERENCE SPEED COMMANDS (LEFT, RIGHT) (m/s)')
    	    self.sheet2.write(3, 4, 'WHEEL SPEED RIGHT (RPM)')
    	    self.sheet2.write(3, 5, 'WHEEL SPEED LEFT (RPM)')
    	    
    	    self.sheet3.write(3, 3, 'GNSS STREAM UNTOUCHED')
    	    self.sheet3.write(3, 4, 'GNSS STREAM RAW')
    	    self.sheet3.write(3, 5, 'GNSS STREAM PARSED')
    	    
    	    self.sheet1.write(3, 3, 'FEELER ANGLES (RIGHT, LEFT)')
    	    
    	    self.sheet1.write(3, 6, 'IMU Euler Angles (heading, roll, pitch)(deg)')
    	    
    	    self.sheet1.write(3, 10, 'Accelerometer (ax,ay,az)(m/s^2)')
    	    
    	    self.sheet1.write(3, 14, 'Linear Accelerometer (ax,ay,az) (m/s^2)')
    	    
    	    self.sheet1.write(3, 18, 'Grav Accelerometer (ax,ay,az) (m/s^2)')
    	    
    	    self.current_row_sheet1=4
    	    self.current_row_sheet2=4
    	    self.current_row_sheet3=4
    	    
    	    self.wb.save(self.current_log_file)
    	    
    	if msg.data==False:
    	    self.logging=False
    	    self.get_logger().info("Stopping data logging under /cornbot/log/%s" %self.unique_filename)
    	    self.wb.save(self.current_log_file)
    	    #self.f.close()
    	    
    	    
def main(args=None):

    rclpy.init(args=args) #initialize a ros2 instance
    daq_node = DAQNode() #initialize the node

    rclpy.spin(daq_node) #set the node in motion

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    daq_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
