cmake_minimum_required(VERSION 2.8.3)
project(automow_node)

find_package(catkin REQUIRED COMPONENTS
        rospy
        geometry_msgs
        diagnostic_msgs
        dynamic_reconfigure
        message_generation
        )


catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        Status.msg
        #   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        Cutters.srv
        #   Service2.srv
)


generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package()