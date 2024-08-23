#Import ROS and ROS Node infrastucture
import rclpy
from rclpy.node import Node

import pygame
import time
import serial
from std_msgs.msg import String
from std_msgs.msg import Bool

#-------------------------------------------------------------------------------------------------------------------------        
class PYGAME_INSTANCE():
	def __init__(self, welcome, debug):
		pygame.init()
		pygame.joystick.init()
		self.joysticks=[]
		self.clock = pygame.time.Clock()
		self.keepPlaying = True
		print()
		
		# for al the connected joysticks
		for i in range(0, pygame.joystick.get_count()):

			# create an Joystick object and add to our list
			self.joysticks.append(pygame.joystick.Joystick(i))
			# print a statement telling what the name of the controller is
			print ("Detected ",self.joysticks[-1].get_name())
			print ()
			# initialize the appended joystick (-1 means last array item)
			self.joysticks[-1].init()
			if self.joysticks[-1].get_init()==True:
				print ("Successfully initialized controller via pygame.")
			elif self.joysticks[-1].get_init()==False:
				print ("Failed to initialized detected controller via pygame.")
			print()	
							
			
			if debug==True:
				self.my_init_debug(i)
			
			if welcome==True:
				print("Hello there controller!")
				print()
				for i in range(0,50):
					self.joysticks[-1].rumble(0,1,100)
					self.joysticks[-1].rumble(1,0,10)
					time.sleep(0.05)
					self.joysticks[-1].stop_rumble()
					
		if len(self.joysticks)==0:
			print("No controller connected")
			print()	
			
	def my_init_debug(self, i):
		print("Num axis: ",self.joysticks[-1].get_numaxes())
			#for i in range(0,joysticks[-1].get_numaxes()):
			#	print(joysticks[-1].get_axis(i))
		print()
		#axis 0 is left joy x from [-1left, 1right]
		#axis 1 is left joy y from [-1up, 1down]
		#axis 2 is right joy x from [-1left, 1right]
		#axis 3 is right joy y from [-1up, 1down] 
		#axis 4 is right trigger from [-1out, 1in]
		#axis 5 is right trigger from [-1out, 1in]
		
		print("Num track pads: ",self.joysticks[-1].get_numballs())
		print()
		
		print("Num buttons: ",self.joysticks[-1].get_numbuttons())
		#for i in range(0,joysticks[-1].get_numbuttons()):
		#	print(joysticks[-1].get_button(i))
		print()
		#button 0 is a
		#button 1 is b
		#button 2 is
		#button 3 is x
		#button 4 is y
		#button 6 is Left bumper
		#button 7 is right bumper
		#button 11 is menu button
		
		#button 13 is left stick
		#button 14 is right stick
		#button 15 is view button
		#button 16 is xbox button
		
		print("Num hats: ",self.joysticks[-1].get_numhats())
		
		print(self.joysticks[i].get_hat(0))
		print()
		#(0,1) is up on D pad
		#(-1,0) is left on D pag
		#(0,-1) is down on D pad
		#(0,1) is right on D pad
		
	def input_handler(self, xbox_teleop):
			
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					self.keepPlaying = False  # Flag that we are done so we exit this loop.
					
				if event.type == pygame.JOYBUTTONDOWN:
					#print("Joystick button pressed.")
					
					if event.button == 0: 
						print('A')
						#joystick = self.joysticks[event.instance_id]
						#if joystick.rumble(0, 0.7, 500):
						#	print(f"Rumble effect played on joystick {event.instance_id}")
					elif event.button == 1:
						print('B')
					elif event.button == 2:
						if self.is_logging==0:
							xbox_teleop.get_logger().info('Beginning logging of GPS/GNSS messages to %s. Press "x" to stop logging points.')
							xbox_teleop.data_logging_callback(True)
							self.is_logging=1
							
						elif self.is_logging==1:
							xbox_teleop.get_logger().info('Stopping logging of GPS/GNSS messages.')
							xbox_teleop.data_logging_callback(False)
							self.is_logging=0
							
					elif event.button == 3:
					
						if self.steer_type==0: #tank steer
							#print("Switching to throttle steer.")
							xbox_teleop.get_logger().info('Switching to throttle steer. Use RT/LT for forward/reverse; left joystick to turn.') 
							self.steer_type=1
							
						elif self.steer_type==1: 
							#print("Switching to tank steer.")
							xbox_teleop.get_logger().info('Switching to tank steer. Left/right joysticks control speed of left/right sides.') 
							self.steer_type=0

					elif event.button == 4:
						print('Left bumper')
						#speed_ref_publi
						#rclpy.shutdown()
					elif event.button == 5:
						print('Right bumper')
					elif event.button == 6:
						print('View')
					elif event.button == 7:
						print('Menu')
					elif event.button == 8:
						print('Left stick')
					elif event.button == 9:
						print('Right stick')
					elif event.button == 10:
						pass
						#print('Xbox Button')
						#xbox_teleop.get_logger().info('Entering protective stop state. Sending <0,0> to motors until killed.')
						#while 1:
						#	xbox_teleop.ref_speed_publish_function(f'<0.0, 0.0>')
					elif event.button == 11:
						pass
					elif event.button == 12:
						pass
					elif event.button == 13:
						pass
					elif event.button == 14:
						pass
					elif event.button == 15:
						pass
					elif event.button == 16:
						pass
						
				if event.type == pygame.JOYAXISMOTION:
					if self.steer_type==0: #tank steer
						#send drive commands normalized to ref speed of [-1.2, 1.2]
						if event.axis == 1:
							#print('Left joystick vertical: '+str(event.value))
							self.left_joy_vert=self.tank_normalize_joy_to_speed_ref(event.value)
							
							xbox_teleop.ref_speed_publish_function(f'<{self.right_joy_vert},{self.left_joy_vert}>')
							
						if event.axis == 4:
							#print('Right Joystick Vertical: '+str(event.value))
							self.right_joy_vert=self.tank_normalize_joy_to_speed_ref(event.value)
							
							xbox_teleop.ref_speed_publish_function(f'<{self.right_joy_vert},{self.left_joy_vert}>')
							
							
					if self.steer_type==1: #throttle steer
						if event.axis == 5: #right trigger for forward
						
							#print('Right trigger: '+str(event.value))
							#print(self.joysticks[-1].get_axis(0))
							
							self.direction=1 #forward
							self.throt_last=event.value #store the last throttle command sense incase user needs to adjust left/right rates when throttle is at full power
							#normalize to reasonable commands using the current value of the horizontal on left joystick
							(self.left_joy_vert, self.right_joy_vert)=self.throt_normalize_joy_to_speed_ref(self.joysticks[-1].get_axis(0), event.value, self.direction) 
							
							xbox_teleop.ref_speed_publish_function(f'<{self.right_joy_vert},{self.left_joy_vert}>')
							#print(event.value)
							
						if event.axis == 0:
							#print('Left joystick horizontal: '+str(event.value))
							"""
							normalize to reasonable commands using the previous throttle value 
							in case it is constant at full, in which case no throttle event will trigger) and 
							the current left hortizontal joystick value under event.value
							"""
							(self.left_joy_vert, self.right_joy_vert)=self.throt_normalize_joy_to_speed_ref(event.value, self.throt_last,self.direction)
							xbox_teleop.ref_speed_publish_function(f'<{self.right_joy_vert},{self.left_joy_vert}>')
							
						if event.axis == 2: #left trigger for reverse
							#print('Left trigger: '+str(event.value))
							#print(self.joysticks[-1].get_axis(0))
							
							self.direction=0 #reverse
							self.throt_last=event.value #store the last throttle command sense incase user needs to adjust left/right rates when throttle is at full power
							#normalize to reasonable commands using the current value of the horizontal on left joystick
							(self.left_joy_vert, self.right_joy_vert)=self.throt_normalize_joy_to_speed_ref(self.joysticks[-1].get_axis(0), event.value, self.direction) 
							
							xbox_teleop.ref_speed_publish_function(f'<{self.right_joy_vert},{self.left_joy_vert}>')
							#print(event.value)

					if event.axis == 3:
						pass
						#print('Right Joystick Horizontal: '+str(event.value))	
					if event.axis == 4:
						pass
						#print('Right trigger: '+str(event.value))
							
					if event.axis == 5:
						#print('Left trigger: '+str(event.value))		
						pass

	def tank_normalize_joy_to_speed_ref(self, value):
		newval=-(5.0505*pow(value,5)-6.3131*pow(value,3)+0.0626*value)
		if -0.05<newval<0.05:
			newval=0
		if newval>1.2:
			newval=1.2
		if newval<-1.2:
			newval=-1.2
		return newval
	
	def throt_normalize_joy_to_speed_ref(self, diff_const, value, direction):
		if direction==1:
			dc=diff_const
			value=value+1
			newval=-0.6*value
			if newval<-1.2:
				newval=-1.2
			if newval>0:
				newval=0.0
			if dc<-0.1:
				newval_right=newval
				newval_left=(20/9)*newval*dc+(11/9)*newval			
			elif dc>0.1:
				newval_left=newval
				newval_right=-(20/9)*newval*dc+(11/9)*newval
			elif -0.1<=dc<=0.1:
				newval_left=newval
				newval_right=newval
			return (newval_left, newval_right)
        	
		elif direction==0:
			dc=diff_const
			value=value+1
			newval=0.6*value
			if newval>1.2:
				newval=1.2
			if newval<0:
				newval=0.0
			if dc<-0.1:
				newval_right=newval
				newval_left=(20/9)*newval*dc+(11/9)*newval	
			elif dc>0.1:
				newval_left=newval
				newval_right=-(20/9)*newval*dc+(11/9)*newval
			elif -0.1<=dc<=0.1:
				newval_left=newval
				newval_right=newval
			return (newval_left, newval_right)	
					
	def main(self, xbox_teleop):	
		self.left_joy_vert=float()
		self.right_joy_vert=float()
		self.steer_type=0 #defaults to tank steer =0, throttle=1
		self.diff_const=0 #for throttle steering left, right speed difference
		self.throt_last=0 #last throttle value
		self.direction=0 #for throttle steering
		self.is_logging=0
		while self.keepPlaying:
			self.clock.tick(60)
			self.input_handler(xbox_teleop)
			
#-------------------------------------------------------------------------------------------------------------------------						
	
class XBoxTeleopNode(Node): #Make a subclass of the Node class called "SpeedRefPublisher"

    def __init__(self):
        super().__init__('xbox_teleop_node') #call the constructor of the class from which this class is inhereted, providing the new node name as "xbox_teleop_node"
        self.publisher_ = self.create_publisher(String, 'cornbot/speed_ref_topic', 10) #declares publisher publishes String messages on the "cornbot/speed_ref_topic" topic, with a max queue size of 10
        self.publisher2_ = self.create_publisher(Bool, 'cornbot/data_logging', 10) #declares publisher publishes String messages on the "cornbot/speed_ref_topic" topic, with a max queue size of 10

    def ref_speed_publish_function(self, string_to_pub):
        msg = String()
        msg.data = string_to_pub
        self.publisher_.publish(msg) #publish the message
        #self.get_logger().info('Publishing new reference speed in m/s (right, left): "%s"' % msg.data) #inform ROS2 console as a ros info level message
        
    def data_logging_callback(self, bool_to_pub):
    	msg = Bool()
    	msg.data = bool_to_pub
    	self.publisher2_.publish(msg)
    	
        
#-------------------------------------------------------------------------------------------------------------------------


	
def main(args=None):

	welcome = True
	debug = False
	p = PYGAME_INSTANCE(welcome, debug)
	
	rclpy.init(args=args) #initialize ROS2 instance
	xbox_teleop = XBoxTeleopNode() #initialize our reference speed publisher
	
	p.main(xbox_teleop)
	
	# If you forget this line, the program will 'hang'
	# on exit if running from IDLE.
	pygame.quit()
