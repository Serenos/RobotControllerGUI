/**
 * @file /include/rqt_bot/main_window.hpp
 *
 * @brief Qt based gui for rqt_bot.
 *
 * @date 2020
 **/
#ifndef rqt_bot_MAIN_WINDOW_H
#define rqt_bot_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include "ui_main_window.h"
#include "qnode.hpp"
#include "addtopics.h"
#include "settings.h"
#include "qrviz.hpp"
//仪表盘头文件
#include "QProcess"
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QSoundEffect>
#include <QComboBox>
#include <QSpinBox>
#include <QVariant>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <map>
//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rqt_bot {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void initUis();
	void initRviz();
	void initTopicList();
	void initVideos();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	

	//button
	void on_button_simplesim_clicked();
	void on_button_gazebo_clicked();
	void on_button_simulation_clicked();
	void on_button_teleop_clicked();
	void on_button_rviz_clicked();

	void on_button_gmap_clicked();
	void on_button_cartographer_clicked();
	void on_button_hector_clicked();
	void on_button_karto_clicked();
	void on_button_frontier_clicked();
	void on_button_savemap_clicked();

	

    /******************************************
    ** Manual connections
    *******************************************/
    //void updateLoggingView(); // no idea why this can't connect automatically
	void slot_state(sensor_msgs::JointState jointstate);
	void slot_odom(double pose_x,double pose_y,double pose_z,double twist_x,double twist_y,double twist_z);
	void on_Slider_raw_valueChanged(int value);
    void on_Slider_linear_valueChanged(int value);
    void slot_cmd_control();
	void slot_rosShutdown();
	//设置界面
    void slot_setting_frame();

	void slot_tab_manage_currentChanged(int);
    void slot_tab_Widget_currentChanged(int);
    void slot_add_topic_btn();
    void slot_choose_topic(QTreeWidgetItem *choose);
    void slot_treewidget_item_value_change(QString);
    void slot_treewidget_item_check_change(int);

	void slot_show_image(int,QImage);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	QRviz *map_rviz=NULL;
    QStandardItemModel* treeView_rviz_model=NULL;
    AddTopics *addtopic_form=NULL;
    //存放rviz treewidget当前显示的控件及控件的父亲的地址
    QMap <QWidget*,QTreeWidgetItem *> widget_to_parentItem_map;
    //存放状态栏的对应关系 display名 状态item
    QMap <QString,QTreeWidgetItem *> tree_rviz_stues;
    //存放display的当前值 item名，参数名称和值
    QMap <QTreeWidgetItem*,QMap<QString,QString>> tree_rviz_values;
    Settings *set=NULL;
    QSoundEffect *media_player=NULL;
};

}  // namespace rqt_bot

#endif // rqt_bot_MAIN_WINDOW_H
