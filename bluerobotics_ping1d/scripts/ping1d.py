#!/usr/bin/env python3

import rospy

from brping import Ping1D

import rospy
from sensor_msgs.msg import FluidPressure 
from sensor_msgs.msg import Range 

def pub():
    pub = rospy.Publisher('Range', Range, queue_size=10)
    rate = rospy.Rate(10)
    _range = Range()
    _range.radiation_type = sound;
    _range.field_of_view = 30.0/180.0*3.1415;

    myPing = Ping1D(_device, _baudrate)
    if myPing.initialize() is False:
        print("Failed to initialize Ping!")
        exit(1)

    print("------------------------------------")
    print("Starting Ping..")
    rospy.loginfo("Starting Ping...")


    while not rospy.is_shutdown():
        data = myPing.get_distance()
        report = {'time_log':'', 'distance':'', 'confidence':''}
        if data:
          _range.header = rospy.get_rostime()
          _range.range = data["distance"]
          rospy.loginfo(_range)
          pub.publish(_range)
          report["time_log"]=str(rospy.Time.now())
          report["distance"]=date["distance"]
          report["confidence"]=data["confidence"]

          report=json.dumps(report)
          message=json.loads(report)
          if _writer is not None
            _writer = writerow(message)

        rate.sleep()

    
if __name__ == '__main__'
    global _device, _baudrate, do_log_raw_data, _writer
    rospy.init_node('ping1d', anonymous = True)
    _device = rospy.get_param("~device", "/dev/ttyUSB0")
    _baudrate = rospy.get_param("~baudrate", 115200)
    do_log_raw_data = rospy.get_param("~do_log_raw_data", False)

    if(do_log_raw_data)
        _file = open('/tmp/ping1d.csv', 'w')
        _writer = csv.writer(_file, fieldnames=fieldnames, extrasaction="ignore", delimiter=' ')
        _writer = writeheader()

    try:
        pub()

    except rospy.ROSINterruptException:
        pass
