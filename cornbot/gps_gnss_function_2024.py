#make the ros node class available for inheritance by our custom node
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float32
from rclpy.clock import Clock #used to synchronize everything to the systemtime


import serial
import time

#for parsing gps/gnss data
from pynmeagps import NMEAReader
from pynmeagps import NMEAMessage
from pygpsclient.helpers import fix2desc, kmph2ms, knots2ms, svid2gnssid

class GNSSNode(Node): #Make a subclass of the Node class called "GNSSNode"

    def __init__(self):
    
    	#call the constructor of the class from which this class is inhereted, 
    	#providing the new node name as "gnss_node"
        super().__init__('gnss_node') 
        self.get_logger().info("Initializing GNSS Node\n")
        
        #this is the serial line that the GPS is sending the messages over
        #prior to starting the simulation you may need to unplug and replug in the GPS/GNSS  
        self.gnss_serial=serial.Serial(port='/dev/ttyGNSSReceiver', baudrate=115200, dsrdtr=True, timeout=0.1) #/dev path is a shortcut to the associated usb device as set by udev rules under /etc/udev/rules.d/99-usb-serial.rules
        time.sleep(2)
        self.nmea_reader=NMEAReader(self.gnss_serial)
        self.get_logger().info("GNSS Serial On: %s\n" % self.gnss_serial.name)
              
        
        #declares publishers on the "cornbot/gnss/..." topics
        self.raw_NMEA_publisher = self.create_publisher(String, 'cornbot/gnss/NMEA/raw_topic', 1) 
        self.readable_NMEA_publisher = self.create_publisher(String, 'cornbot/gnss/NMEA/parsed_topic', 1)
        self.message_type_pub = self.create_publisher(String, 'cornbot/gnss/NMEA/msg_types_topic', 1)
        self.message_types=set() #used to track the types of NMEA sentences we have processed
             
        self.gnss_time_pub=self.create_publisher(String, 'cornbot/gnss/gnss_time', 1)
        
        self.hdop_pub=self.create_publisher(PointStamped, 'cornbot/gnss/dop/hdop_topic',1)
        self.pdop_pub=self.create_publisher(PointStamped, 'cornbot/gnss/dop/pdop_topic',1)
        self.vdop_pub=self.create_publisher(PointStamped, 'cornbot/gnss/dop/vdop_topic',1)
        
        self.altitude_pub=self.create_publisher(String, 'cornbot/gnss/positioning/altitude',1)
        self.sep_pub=self.create_publisher(String, 'cornbot/gnss/positioning/geoidal_separation',1)
        self.sog_pub=self.create_publisher(PointStamped, 'cornbot/gnss/positioning/speed_over_ground_knots',1)
        self.sogkm_pub=self.create_publisher(PointStamped, 'cornbot/gnss/positioning/speed_over_ground_kmh',1)
        self.cog_pub=self.create_publisher(PointStamped, 'cornbot/gnss/positioning/course_over_ground',1)
        self.ct_pub=self.create_publisher(PointStamped, 'cornbot/gnss/positioning/course_true',1)
        self.ctm_pub=self.create_publisher(PointStamped, 'cornbot/gnss/positioning/course_mag',1)
        self.lat_lon_stamped_pub = self.create_publisher(PointStamped, 'cornbot/gnss/positioning/lat_lon_stamped_topic', 1)
        self.lat_lon_pub = self.create_publisher(Point, 'cornbot/gnss/positioning/lat_lon_topic', 1)
        
        self.status_pub=self.create_publisher(String, 'cornbot/gnss/status/status_topic',1)
        self.fix_quality_pub=self.create_publisher(String, 'cornbot/gnss/status/fix_quality_topic',1)
        self.pos_mode_pub=self.create_publisher(String, 'cornbot/gnss/status/pos_mode_topic',1)
        self.op_mode_pub=self.create_publisher(String, 'cornbot/gnss/status/op_mode_topic', 1)
        self.nav_mode_pub=self.create_publisher(String,'cornbot/gnss/status/nav_mode_topic',1)
        self.sys_id_pub=self.create_publisher(String, 'cornbot/gnss/status/system_id_topic',1)   
        
        self.prn_codes_pub=self.create_publisher(String, 'cornbot/gnss/sv_info/prn_codes_topic',1) 
        self.num_sats_pub=self.create_publisher(String, 'cornbot/gnss/sv_info/num_SVs_topic',1)    
        self.svid_publisher = self.create_publisher(String, 'cornbot/gnss/sv_info/svids_topic', 1)
        self.elevation_publisher = self.create_publisher(String, 'cornbot/gnss/sv_info/elevations_topic', 1)
        self.azimuth_publisher = self.create_publisher(String, 'cornbot/gnss/sv_info/azimuths_topic', 1)
        self.cno_publisher = self.create_publisher(String, 'cornbot/gnss/sv_info/cnos_topic', 1)
        self.satellite_info_publisher = self.create_publisher(String, 'cornbot/gnss/sv_info/raw_sv_topic', 1)
        self.avg_cno_publisher=self.create_publisher(Float32, 'cornbot/gnss/sv_info/ave_cno_topic',1)
        self.signalID_pub=self.create_publisher(String, 'cornbot/gnss/sv_info/signal_type',1)
        
        self.get_logger().info("Publishing to various topics under /cornbot/gnss/...\n")
        time.sleep(2)
        timer_period = 0.005 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):           
        #print('In waiting:'+str(self.gnss_serial.inWaiting()))
        if self.gnss_serial.inWaiting()>0 and self.gnss_serial.inWaiting()<4095: #if our input buffer has content but isn't full
            try:
                #nmea_data=self.gnss_serial.readline().decode('utf-8')    
                raw_data, parsed_data=self.nmea_reader.read()
                
                if raw_data!=None and parsed_data!=None: #if the raw data and the parsed data are populated
                
                    #Publish the raw gnss data
                    raw_nmea_msg = String()	
                    raw_nmea_msg.data = str(raw_data)
                    self.raw_NMEA_publisher.publish(raw_nmea_msg) 
                    #inform ROS2 console as a ros info level message
                    #self.get_logger().info('Raw NMEA string: %s' % raw_nmea_msg.data)
	                       
                    # Publish the readable message
                    readable_msg = String()
                    readable_msg.data = str(parsed_data)
                    self.readable_NMEA_publisher.publish(readable_msg)
                    #self.get_logger().info('Readable NMEA string: %s' % readable_msg.data)
                    
                    msg_type=str(parsed_data.talker+parsed_data.msgID)
                    if msg_type not in self.message_types: #publish an updated list of message types if a new message type is added
                        self.message_types.add(msg_type)
                        # Publish the updated list of message types
                    message_type_msg = String()
                    message_type_msg.data = ','.join(sorted(self.message_types))
                    self.message_type_pub.publish(message_type_msg)

                    #_____________________________________________________________________________________________________________
                    if str(parsed_data.msgID)=='GGA': #example <NMEA(GNGGA, time=19:18:14.250000, lat=43.8826341333, NS=N, lon=-72.1578000683, EW=W, quality=4, numSV=12, HDOP=0.57, alt=143.185, altUnit=M, sep=-32.547, sepUnit=M, diffAge=, diffStation=0)>
                        self.lat=parsed_data.lat #
                        self.lon=parsed_data.lon  #     
                        self.hdop=parsed_data.HDOP #
                        self.fix_quality=parsed_data.quality #                                        
                        self.gnss_time=parsed_data.time #
                        self.num_sv=parsed_data.numSV #
                        self.altitude=parsed_data.alt #
                        self.altUnit=parsed_data.altUnit #
                        self.sep=parsed_data.sep #
                        self.sepUnit=parsed_data.sepUnit #
                        self.diffAge=parsed_data.diffAge #UNPUBLISHED
                        self.diffStation=parsed_data.diffStation #UNPUBLISHED
                                                
                        #PUBLISH GGA DATA ---------------------------------------
                        sep_msg=String()
                        sep_msg.data=str(self.sep)+" "+str(self.sepUnit)
                        self.sep_pub.publish(sep_msg)
                        
                        alt_msg=String()
                        alt_msg.data=str(self.altitude)+" "+str(self.altUnit)
                        self.altitude_pub.publish(alt_msg)
                        
                        numSV_msg=String()
                        numSV_msg.data=str(self.num_sv)
                        self.num_sats_pub.publish(numSV_msg)
                        
                        gnss_time_msg=String()
                        gnss_time_msg.data=str(self.gnss_time)
                        self.gnss_time_pub.publish(gnss_time_msg)
                        
                        lat_lon_msg=PointStamped()
                        lat_lon_msg.header.stamp=Clock().now().to_msg()
                        lat_lon_msg.header.frame_id='/gnss'
                        lat_lon_msg.point.x=self.lat
                        lat_lon_msg.point.y=self.lon
                        self.lat_lon_stamped_pub.publish(lat_lon_msg)
                        
                        hdop_msg=PointStamped()
                        hdop_msg.header.stamp=Clock().now().to_msg()
                        hdop_msg.header.frame_id='/gnss'
                        hdop_msg.point.x=self.hdop
                        self.hdop_pub.publish(hdop_msg)
                        
                        qual_msg=String()
                        if self.fix_quality==0:
                            qual_msg.data='Fix not valid'
                        elif self.fix_quality==1:
                            qual_msg.data='GPS Fix'
                        elif self.fix_quality==2:
                            qual_msg.data='Differential GPS by SBAS'
                        elif self.fix_quality==3:
                            qual_msg.data='Type 3, Not Applicable'
                        elif self.fix_quality==4:
                            qual_msg.data='RTK Fixed, xFill'
                        elif self.fix_quality==5:
                            qual_msg.data='RTK Float, OmniSTAT XP/HP, Location RTK, RTX'
                        elif self.fix_quality==6:
                            qual_msg.data='INS Dead reckoning'
                        else:
                            qual_msg.data='Fix Quality Unknown'
                        self.fix_quality_pub.publish(qual_msg)



                    #___________________________________________________________________________________________________________________
                    elif str(parsed_data.msgID) == 'RMC': #example <NMEA(GNRMC, time=19:18:13.750000, status=A, lat=43.8826344017, NS=N, lon=-72.1577986133, EW=W, spd=0.705, cog=343.0, date=2023-09-05, mv=, mvEW=, posMode=R, navStatus=V)>
                        self.lat = parsed_data.lat #
                        self.lon = parsed_data.lon #
                        self.gnss_time = parsed_data.time #
                        self.status = parsed_data.status #
                        self.speed_over_ground_knots = parsed_data.spd #
                        self.course_over_ground = parsed_data.cog #
                        self.date = parsed_data.date #UNPUBLISHED
                        self.mag_variation = parsed_data.mv #UNPUBLISHED
                        self.mag_var_direction = parsed_data.mvEW #UNPUBLISHED
                        self.mode_indicator = parsed_data.posMode #
                        self.nav_status=parsed_data.navStatus #UNPUBLISHED
                        
                        #PUBLISH RMC DATA ---------------------------------------

                        
                        sog_msg=PointStamped()
                        sog_msg.header.stamp=Clock().now().to_msg()
                        sog_msg.header.frame_id='/gnss'
                        sog_msg.point.x=self.speed_over_ground_knots
                        self.sog_pub.publish(sog_msg)
                        
                        cog_msg=PointStamped()
                        cog_msg.header.stamp=Clock().now().to_msg()
                        cog_msg.header.frame_id='/gnss'
                        if self.course_over_ground!='':
                            cog_msg.point.x=self.course_over_ground
                            self.cog_pub.publish(cog_msg)
                        
                        gnss_time_msg=String()
                        gnss_time_msg.data=str(self.gnss_time)
                        self.gnss_time_pub.publish(gnss_time_msg)
                        
                        posMode_msg=String()
                        if self.mode_indicator=='D':
                            posMode_msg.data='Differential Positioning - RMC'
                        elif self.mode_indicator=='A':
                            posMode_msg.data='Autonomous Positioning - RMC'
                        elif self.mode_indicator=='E':
                            posMode_msg.data='Estimated (Dead reckoning) Positioning - RMC'
                        elif self.mode_indicator=='N':
                            posMode_msg.data='Invalid Data - RMC'
                        elif self.mode_indicator=='R':
                            posMode_msg.data='RTK Fix Positioning - RMC'
                        elif self.mode_indicator=='M':
                            posMode_msg.data='Manual input - RMC'
                        elif self.mode_indicator=='S':
                            posMode_msg.data='Simulator Mode - RMC'
                        else:
                            posMode_msg.data='Unknown Positioning Mode - RMC'
                        self.pos_mode_pub.publish(posMode_msg)
                        
                        status_msg=String()
                        if self.status=='A':
                            status_msg.data='Active'
                        elif self.status=='V':
                            status_msg.data='Void'
                        else:
                            status_msg.data='Status Unknown'
                        self.status_pub.publish(status_msg)
                        
                        lat_lon_msg=PointStamped()
                        lat_lon_msg.header.stamp=Clock().now().to_msg()
                        lat_lon_msg.header.frame_id='/gnss'
                        lat_lon_msg.point.x=self.lat
                        lat_lon_msg.point.y=self.lon
                        self.lat_lon_stamped_pub.publish(lat_lon_msg)
                        
                        lat_lon=Point()
                        lat_lon.x=self.lat
                        lat_lon.y=self.lon
                        self.lat_lon_pub.publish(lat_lon)
                     
                     
                    #__________________________________________________________________________________                       
                    elif str(parsed_data.msgID) == 'GLL': #example <NMEA(GNGLL, lat=43.8826337233, NS=N, lon=-72.157797655, EW=W, time=19:18:13, status=A, posMode=D)>
                        self.lat = parsed_data.lat #
                        self.lon = parsed_data.lon #
                        self.gnss_time = parsed_data.time #
                        self.status = parsed_data.status #
                        self.mode_indicator = parsed_data.posMode  #                      
                        
                        #PUBLISH GLL DATA ---------------------------------------
                        gnss_time_msg=String()
                        gnss_time_msg.data=str(self.gnss_time)
                        self.gnss_time_pub.publish(gnss_time_msg)
                        
                        posMode_msg=String()
                        if self.mode_indicator=='D':
                            posMode_msg.data='Differential Positioning - GLL'
                        elif self.mode_indicator=='A':
                            posMode_msg.data='Autonomous Positioning - GLL'
                        elif self.mode_indicator=='E':
                            posMode_msg.data='Estimated (Dead reckoning) Positioning - GLL'
                        elif self.mode_indicator=='N':
                            posMode_msg.data='Invalid Data - GLL'
                        elif self.mode_indicator=='R':
                            posMode_msg.data='RTK Fix Positioning - GLL'
                        elif self.mode_indicator=='M':
                            posMode_msg.data='Manual input - GLL'
                        elif self.mode_indicator=='S':
                            posMode_msg.data='Simulator Mode - GLL'
                        else:
                            posMode_msg.data='Unknown Positioning Mode - GLL'
                        self.pos_mode_pub.publish(posMode_msg)
                        
                        status_msg=String()
                        if self.status=='A':
                            status_msg.data='Active'
                        elif self.status=='V':
                            status_msg.data='Void'
                        else:
                            status_msg.data='Status Unknown'
                        self.status_pub.publish(status_msg)
                        
                        lat_lon_msg=PointStamped()
                        lat_lon_msg.header.stamp=Clock().now().to_msg()
                        lat_lon_msg.header.frame_id='/gnss'
                        lat_lon_msg.point.x=self.lat
                        lat_lon_msg.point.y=self.lon
                        self.lat_lon_stamped_pub.publish(lat_lon_msg)
                        
                        lat_lon=Point()
                        lat_lon.x=self.lat
                        lat_lon.y=self.lon
                        self.lat_lon_pub.publish(lat_lon)
                        
                        
                    #__________________________________________________________________________________   
                    elif str(parsed_data.msgID) == 'GSA': #example <NMEA(GNGSA, opMode=A, navMode=3, svid_01=46, svid_02=44, svid_03=24, svid_04=18, svid_05=10, svid_06=12, svid_07=23, svid_08=15, svid_09=32, svid_10=, svid_11=, svid_12=, PDOP=1.15, HDOP=0.6, VDOP=0.98, systemId=1)>
                        self.op_mode = parsed_data.opMode #
                        self.fix_type = parsed_data.navMode #
                        self.system_id = parsed_data.systemId #
                        self.satellites = [parsed_data.svid_01, parsed_data.svid_02, parsed_data.svid_03, parsed_data.svid_04, parsed_data.svid_05, parsed_data.svid_06, parsed_data.svid_07, parsed_data.svid_08, parsed_data.svid_09, parsed_data.svid_10, parsed_data.svid_11, parsed_data.svid_12] #
                        self.pdop = parsed_data.PDOP #
                        self.hdop = parsed_data.HDOP #
                        self.vdop = parsed_data.VDOP #
                        
                        #PUBLISH GSA DATA ---------------------------------------
                        prn_msg=String()
                        prn_msg.data=str(self.satellites)
                        self.prn_codes_pub.publish(prn_msg)
                        
                        sysID_msg=String()
                        if self.system_id==1:
                            sysID_msg.data='GPS'
                        elif self.system_id==2:
                            sysID_msg.data='GLONASS'
                        elif self.system_id==3:
                            sysID_msg.data='Galileo'
                        elif self.system_id==4:
                            sysID_msg.data='BeiDou'
                        elif self.system_id==0:
                            sysID_msg.data='QZSS'
                        else:
                            sysID_msg.data='Unknown Constellation'
                        self.sys_id_pub.publish(sysID_msg)
                        
                        navMode_msg=String()
                        if self.fix_type==1:
                            navMode_msg.data='Fix type not available'
                        elif self.fix_type==2:
                            navMode_msg.data='2D Fix Type'
                        elif self.fix_type==3:
                            navMode_msg.data='3D Fix Type'
                        else:
                            navMode_msg.data='Unknown Fix Type'
                        self.nav_mode_pub.publish(navMode_msg)
                        
                        opMode_msg=String()
                        if self.op_mode=='A':
                            opMode_msg.data='Automatic'
                        elif self.op_mode=='M':
                            opMode_msg.data='Manual'
                        else:
                            opMode_msg.data='Unknown OpMode'
                        self.op_mode_pub.publish(opMode_msg)
                        
                        hdop_msg=PointStamped()
                        hdop_msg.header.stamp=Clock().now().to_msg()
                        hdop_msg.header.frame_id='/gnss'
                        hdop_msg.point.x=self.hdop
                        self.hdop_pub.publish(hdop_msg)
                        
                        pdop_msg=PointStamped()
                        pdop_msg.header.stamp=Clock().now().to_msg()
                        pdop_msg.header.frame_id='/gnss'
                        pdop_msg.point.x=self.pdop
                        self.pdop_pub.publish(pdop_msg)

                        vdop_msg=PointStamped()
                        vdop_msg.header.stamp=Clock().now().to_msg()
                        vdop_msg.header.frame_id='/gnss'
                        vdop_msg.point.x=self.vdop
                        self.vdop_pub.publish(vdop_msg)                        
                        
                        
                    #______________________________________________________________________________________    
                    elif str(parsed_data.msgID) == 'VTG': #example <NMEA(GNVTG, cogt=258.34, cogtUnit=T, cogm=, cogmUnit=M, sogn=0.985, sognUnit=N, sogk=1.824, sogkUnit=K, posMode=D)>
                        self.course_true = parsed_data.cogt #
                        self.course_true_unit=parsed_data.cogtUnit #
                        self.course_magnetic = parsed_data.cogm #
                        self.course_magnetic_unit=parsed_data.cogmUnit #
                        self.speed_knots = parsed_data.sogn #
                        self.speed_knots_unit = parsed_data.sognUnit #
                        self.speed_kmh = parsed_data.sogk #
                        self.speed_kmh_units = parsed_data.sogkUnit #
                        self.mode_indicator = parsed_data.posMode #
                        
                        #PUBLISH VTG DATA ------------------------------------
                        ctm_msg=PointStamped()
                        ctm_msg.header.stamp=Clock().now().to_msg()
                        ctm_msg.header.frame_id='/gnss'
                        if self.course_magnetic != '':
                            ctm_msg.point.x=self.course_magnetic
                            self.ctm_pub.publish(ctm_msg)
                        
                        ct_msg=PointStamped()
                        ct_msg.header.stamp=Clock().now().to_msg()
                        ct_msg.header.frame_id='/gnss'
                        if self.course_true!='':
                            ct_msg.point.x=self.course_true
                            self.ct_pub.publish(ct_msg)
                        
                        sogkm_msg=PointStamped()
                        sogkm_msg.header.stamp=Clock().now().to_msg()
                        sogkm_msg.header.frame_id='/gnss'
                        sogkm_msg.point.x=s=float(self.speed_kmh)
                        self.sogkm_pub.publish(sogkm_msg)
                        
                        sog_msg=PointStamped()
                        sog_msg.header.stamp=Clock().now().to_msg()
                        sog_msg.header.frame_id='/gnss'
                        sog_msg.point.x=float(self.speed_knots)
                        self.sog_pub.publish(sog_msg)
                        
                        posMode_msg=String()
                        if self.mode_indicator=='D':
                            posMode_msg.data='Differential Positioning - VTG'
                        elif self.mode_indicator=='A':
                            posMode_msg.data='Autonomous Positioning - VTG'
                        elif self.mode_indicator=='E':
                            posMode_msg.data='Estimated (Dead reckoning) Positioning - VTG'
                        elif self.mode_indicator=='N':
                            posMode_msg.data='Invalid Data - VTG'
                        elif self.mode_indicator=='R':
                            posMode_msg.data='RTK Fix Positioning - VTG'
                        elif self.mode_indicator=='M':
                            posMode_msg.data='Manual input - VTG'
                        elif self.mode_indicator=='S':
                            posMode_msg.data='Simulator Mode - VTG'
                        else:
                            posMode_msg.data='Unknown Positioning Mode - VTG'
                        self.pos_mode_pub.publish(posMode_msg)
                        
                        
                        
                        
                        
                    elif str(parsed_data.msgID) == 'GSV': #example <NMEA(GPGSV, numMsg=2, msgNum=1, numSV=8, svid_01=10, elv_01=59.0, az_01=312, cno_01=41, svid_02=12, elv_02=12.0, az_02=120, cno_02=33, svid_03=15, elv_03=25.0, az_03=74, cno_03=34, svid_04=18, elv_04=29.0, az_04=190, cno_04=40, signalID=6)>
                        self.num_messages = parsed_data.numMsg
                        self.message_number = parsed_data.msgNum
                        self.signalID=parsed_data.signalID
                        
                        signalID_msg=String()
                        if self.signalID==0:
                            signalID_msg.data='L1 C/A'
                        elif self.signalID==1:
                            signalID_msg.data='L1 P(Y)'
                        elif self.signalID==2:
                            signalID_msg.data='L2 P(Y)'
                        elif self.signalID==3:
                            signalID_msg.data='L2C (M+L)'                            
                        elif self.signalID==4:
                            signalID_msg.data='L5'                            
                        elif self.signalID==5:
                            signalID_msg.data='Reserved'                            
                        elif self.signalID==6:
                            signalID_msg.data='L1 C/A, L2C'
                        self.signalID_pub.publish(signalID_msg)
                                                    
                        if self.message_number == 1:
                            self.satellite_info = []
                            self.svids = []
                            self.elevations = []
                            self.azimuths = []
                            self.cnos = []

                        for i in range(1, 5):
                            svid = getattr(parsed_data, f'svid_{i:02}', None)
                            elevation = getattr(parsed_data, f'elv_{i:02}', None)
                            azimuth = getattr(parsed_data, f'az_{i:02}', None)
                            cno = getattr(parsed_data, f'cno_{i:02}', None)
                            if svid is not None:
                                self.satellite_info.append({'svid': svid, 'elevation': elevation, 'azimuth': azimuth, 'cno': cno})
                                self.svids.append(svid)
                                self.elevations.append(elevation)
                                self.azimuths.append(azimuth)
                                self.cnos.append(cno)

                        if self.message_number == self.num_messages:
                            # Publish svids
                            svid_msg = String()
                            svid_msg.data = json.dumps(self.svids)
                            self.svid_publisher.publish(svid_msg)

                            # Publish elevations
                            elevation_msg = String()
                            elevation_msg.data = json.dumps(self.elevations)
                            self.elevation_publisher.publish(elevation_msg)

                            # Publish azimuths
                            azimuth_msg = String()
                            azimuth_msg.data = json.dumps(self.azimuths)
                            self.azimuth_publisher.publish(azimuth_msg)

                            # Publish cnos
                            cno_msg = String()
                            cno_msg.data = json.dumps(self.cnos)
                            self.cno_publisher.publish(cno_msg)
                            
                            #Publish a single average cno
                            if all(cno != '' for cno in self.cnos) and len(self.cnos)!=0:
                                
                                avg_cno = sum(float(cno) for cno in self.cnos) / len(self.cnos)
                                avg_cno_msg = Float32()
                                avg_cno_msg.data = avg_cno
                                self.avg_cno_publisher.publish(avg_cno_msg)

                            # Publish satellite info as JSON string
                            satellite_info_msg = String()
                            satellite_info_msg.data = json.dumps(self.satellite_info)
                            self.satellite_info_publisher.publish(satellite_info_msg)

                            # Reset the satellite information for the next sequence
                            self.satellite_info = []		        
            
            except KeyboardInterrupt:
                time.sleep(2)
                self.get_logger().fatal("\nKeyboard interrupt during serial read of GNSS data. Killing node. Closing serial.\n")
                time.sleep(2)
                self.gnss_serial.close()
                exit()
            except UnicodeDecodeError:
                self.get_logger().error("Unicode decode error")
            except Exception as e:
                self.get_logger().error('Error: %s' % e)
            
            
        elif self.gnss_serial.inWaiting() > 4094:
            self.get_logger().warn("GNSS serial input buffer overload. Flushing input buffer\n")
            self.gnss_serial.flushInput()
            
def main(args=None):

    rclpy.init(args=args) #initialize a ros2 instance
    gnss_node = GNSSNode() #initialize the node

    try:
        rclpy.spin(gnss_node) #set the node in motion
    except KeyboardInterrupt:
        time.sleep(2)
        gnss_node.get_logger().fatal("Keyboard interrupt of ROS2 spin on GNSS node. Killing node. Closing serial.")
        gnss_node.gnss_serial.close()
        gnss_node.destroy_node()
        rclpy.shutdown()
        time.sleep(2)
        exit()

if __name__ == '__main__':
    main()
