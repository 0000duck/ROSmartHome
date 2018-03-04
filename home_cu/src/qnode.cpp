/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/home_cu/qnode.hpp"
#include <std_msgs/Int32.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace home_cu {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"home_cu");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    lights_subscriber = n.subscribe<std_msgs::Int32>("lights", 1000, &QNode::switchLight, this);
    chatter_publisher = n.advertise<std_msgs::Int32>("lights_status", 1000);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"home_cu");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    lights_subscriber = n.subscribe<std_msgs::Int32>("lights", 1000, &QNode::switchLight, this);
    chatter_publisher = n.advertise<std_msgs::Int32>("lights_status", 1000);
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(1); // 1 hz = 1 loop by second.
    while( ros::ok()) {
        ros::spinOnce();    // check for message arrival
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::switchLight(const std_msgs::Int32::ConstPtr &msg)
{
    Q_EMIT switchLight((int)msg.get()->data);

    std::stringstream ss;
    ss << "I heard: switch light " << msg.get()->data;
    log(Info, ss.str());
}

void QNode::informStatus(int light, bool status)
{
    std_msgs::Int32 msg;
    msg.data = (light & 65535) | ((int)status << 16);
    chatter_publisher.publish(msg);

    std::stringstream ss;
    ss << "I sent: light " << light << " is " << status;
    log(Info, ss.str());
}

}  // namespace home_cu
