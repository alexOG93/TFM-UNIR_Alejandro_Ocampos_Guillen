#!/usr/bin/env python

import rospy, serial
from std_msgs.msg import String
from sensor_msgs.msg import Range

# Robot will stop when lower distances are detected
MINIMUM_DISTANCE = 20.0
COM_RATE = 20

dist_F = 0.0
dist_R = 0.0
dist_L = 0.0
dist_B = 0.0


def obs_avoid():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()    
    
    # Publisher
    pub_obs_avoid = rospy.Publisher('obstacle_avoidance', String, queue_size=1)
    msg_to_publish  = 0

    rate = rospy.Rate(COM_RATE)
    print("obstacle_avoid_ctrl node is running succesfully, press Ctrl+C to exit.")    
    
    while not rospy.is_shutdown():
        
        line = ser.readline().decode('utf-8').rstrip()
        # ~ print(line)
        # ~ ser.flush()   
        # For now, only front distance is checked
        measures = line.split(",")
        cells_state = []
        try:
            for measure in measures:
                if int(measure) < MINIMUM_DISTANCE:
                    cells_state.append("o") # Cell occupied
                else:
                    cells_state.append("f") # Cell free
            # ~ cells_state.append(measures[4])
        except ValueError:
            cells_state = "oooo"
        except IndexError:
            cells_state = "oooo"
        
        pub_obs_avoid.publish("".join(cells_state))
        
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("obstacle_avoid_ctrl")
    obs_avoid()
