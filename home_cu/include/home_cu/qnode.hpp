/**
 * @file /include/home_cu/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef home_cu_QNODE_HPP_
#define home_cu_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Int32.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace home_cu {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    void switchLight(const std_msgs::Int32::ConstPtr& msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void switchLight(int light);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
    ros::Subscriber lights_subscriber;
};

}  // namespace home_cu

#endif /* home_cu_QNODE_HPP_ */
