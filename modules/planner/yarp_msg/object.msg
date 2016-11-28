#geometry_msgs/Point point
#yarp/point3D position
#yarp/dimension dimension

# ROS message of 3D object, because YARP only provide std_msgs type of message, have to use ONLY them to form the message of 2 type

string objectType	# target or obstacle
float64[] position	# position of center, expected 3 members 
float64[] dimension	# dimension of object, expected 3 members
