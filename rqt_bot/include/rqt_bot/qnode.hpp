/**
 * @file /include/rqt_bot/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date 2020
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rqt_bot_QNODE_HPP_
#define rqt_bot_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QObject>
#include <QtCore>
#include <QStringListModel>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式

#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rqt_bot {

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
	void move_base(char k,float speed_linear,float speed_trun);
	void Sub_Image(QString topic,int frame_id);
	QImage Mat2QImage(cv::Mat const& src);
	QMap<QString,QString> get_topic_list();
	

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

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
	void state(sensor_msgs::JointState jointstate);
	void odom(double pose_x, double pose_y, double pose_z, double twist_x, double twist_y, double twist_z);
	void power();
	void Show_Image(int,QImage);
	void Master_shutdown();

private:
	int init_argc;
	char** init_argv;
	//Publisher
	ros::Publisher chatter_publisher;
	ros::Publisher cmd_pub;

	//Subscriber
	ros::Subscriber jointState_sub;
	ros::Subscriber odom_sub;

	//图像订阅
    image_transport::Subscriber image_sub0;
    image_transport::Subscriber image_sub1;
	image_transport::Subscriber image_sub2;
    image_transport::Subscriber image_sub3;
	//图像format
    QString video0_format;
    QString video1_format;
	QString video2_format;
    QString video3_format;

    QStringListModel logging_model;

	void stateCallback(const sensor_msgs::JointState &state);
	void odomCallback(const nav_msgs::Odometry &odometry);
	void imageCallback0(const sensor_msgs::ImageConstPtr& msg);
	void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
	void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
	void imageCallback3(const sensor_msgs::ImageConstPtr& msg);
};

}  // namespace rqt_bot

#endif /* rqt_bot_QNODE_HPP_ */
