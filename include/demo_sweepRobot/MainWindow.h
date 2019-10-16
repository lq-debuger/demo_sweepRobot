//
// Created by lq on 2019/10/14.
//

#ifndef DEMO_SWEEPROBOT_MAINWINDOW_H
#define DEMO_SWEEPROBOT_MAINWINDOW_H

#include "ros/ros.h"
#include <QtWidgets>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <thread>
#include "std_msgs/Float64.h"

using namespace std;
class MainWindow :public QWidget{
public:
    MainWindow(ros::NodeHandle &n,QWidget *parent= nullptr);

    ~MainWindow();

private:
    ros::Publisher velpublisher;
    ros::Subscriber subscriber;
    //给rqt_plot发送数据
    ros::Publisher plotpublisher;
    double srcX;
    double srcY;
    double turtle_angular;
    double desX;
    double desY;
    double kp;
    double kd;
    double ki;
    //the symbol of stop
    bool stopFlag = 0;
    QTimer *updateTimer;
    ros::NodeHandle node;
    QFormLayout *layout;
    QLineEdit *lineX;
    QLineEdit *lineY;
    QPushButton *btnDo;
    QLabel * labelX;
    QLabel * labelY;
    QLabel * labelTheta;
    QLineEdit *linekp;
    QLineEdit *lineki;
    QLineEdit *linekd;
    //竖直
    QPushButton *btnSweepV;
    //水平
    QPushButton *btnSweepH;
    //回形
    QPushButton *btnSweepR;
    //终止运动按钮
    QPushButton *btnStop;


private:
    void initUI();

    void initRos();

    void clickDo();

    void onUpdate();

    void poseCallback(const turtlesim::Pose::ConstPtr &message);

    //加算距离
    double calcuDistance(double srcX, double srcY, double desX, double desY);

    //计算角度
    double calcuAngular(double srcX, double srcY, double desX, double desY);

    //移动乌龟
    int moveTurtle();

    void clickSweepV();

    //竖直扫地
    void sweepV();

    void clickSweepH();

    //水平扫地
    void sweepH();

    void clickSweepR();

    //回形扫地
    void sweepR();

    void clickStop();
};


#endif //DEMO_SWEEPROBOT_MAINWINDOW_H
