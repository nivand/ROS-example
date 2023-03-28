#!/usr/bin/env python
import rospy
from delf.msg import SensorInformation
from hrwros_utilities.sim_sensor_data import distSensorData as getSensorData
# #
def SensorInfoPublisher():
    si_publisher=rospy.Publisher("sensor_info",SensorInformation,queue_size= 10)
    rospy.init_node('sensor_info_publisher')
    rate=rospy.Rate(10)#10hz
# create a new  sensor information object and fill in its contenets.
    sensor_info=SensorInformation()
# fill in the header information .
    sensor_info.sensor_data.header.stamp=rospy.Time.now()
    sensor_info.sensor_data.header.frame_id='distance_sensor_frame'
# fill in the sensor data information .
    sensor_info.sensor_data.radiation_type=sensor_info.sensor_data.ULTRASOUND
    sensor_info.sensor_data.field_of_view=0.5 #field_of_view of sensor in rad.
    sensor_info.sensor_data.min_range=0.02 #distance in m.
    sensor_info.sensor_data.max_range=2.00 #distance in m.
#fill in the manufacture name and part number.
    sensor_info.maker_name='the ULTRASOUND company'
    sensor_info.part_number=123456
    while not rospy.is_shutdown():
        #read sensor datafrom simulated sensor
        sensor_info.sensor_data.range=getSensorData(sensor_info.sensor_data.radiation_type,
        sensor_info.sensor_data.min_range,sensor_info.sensor_data.max_range)
        #publish the sensor Sensorinformation on the sensor_info topic
        si_publisher.publish(sensor_info)
        #print a log message if all went well
        rospy.loginfo('all went well')
        rate.sleep()
if __name__=='__main__':
#lets call the correct fun here
    try:
        SensorInfoPublisher()
    except rospy.ROSInterruptException:
        pass
