
#include "MainWindow.h"

MainWindow::MainWindow(ros::NodeHandle &n, QWidget *parent):QWidget(parent),node(n) {

    initUI();
    initRos();
}

MainWindow::~MainWindow() {}

void MainWindow::initUI() {
    updateTimer = new QTimer(this);
    updateTimer->setInterval(16);
    updateTimer->start();
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::onUpdate);

    setWindowTitle("扫地机器人");
    layout = new QFormLayout();
    setLayout(layout);
    lineX = new QLineEdit("10.544445");
    layout->addRow("X坐标",lineX);
    lineY = new QLineEdit("5.544445");
    layout->addRow("Y坐标", lineY);
    linekp = new QLineEdit("0.2");
    layout->addRow("kp:", linekp);
    lineki = new QLineEdit("0.2");
    layout->addRow("ki:", lineki);
    linekd = new QLineEdit("0.3");
    layout->addRow("kd:", linekd);
    btnDo = new QPushButton("执行");
    layout->addWidget(btnDo);
    btnSweepV = new QPushButton("竖向扫地");
    layout->addRow(btnSweepV);
    btnSweepH = new QPushButton("横向扫地");
    layout->addRow(btnSweepH);
    btnSweepR = new QPushButton("回形扫地");
    layout->addRow(btnSweepR);
    btnStop = new QPushButton("终止运动");
    layout->addRow(btnStop);
    labelX = new QLabel();
    layout->addRow("X坐标", labelX);
    labelY = new QLabel();
    layout->addRow("Y坐标", labelY);
    labelTheta = new QLabel();
    layout->addRow("角度:", labelTheta);


    connect(btnDo,&QPushButton::clicked,this,&MainWindow::clickDo);
    connect(btnSweepV,&QPushButton::clicked,this,&MainWindow::clickSweepV);
    connect(btnSweepH,&QPushButton::clicked,this,&MainWindow::clickSweepH);
    connect(btnSweepR,&QPushButton::clicked,this,&MainWindow::clickSweepR);
    connect(btnStop,&QPushButton::clicked,this,&MainWindow::clickStop);
}

void MainWindow::initRos() {
    //线速度publisher
    string topicName = "/turtle1/cmd_vel";
    velpublisher = node.advertise<geometry_msgs::Twist>(topicName, 1000);

    //plot publisher
    string topicName2 = "linear/vel";
    plotpublisher = node.advertise<std_msgs::Float64>(topicName2, 1000);

    //pose订阅
    string topicName1 = "/turtle1/pose";
    //获取srcX,srcY的值
    subscriber = node.subscribe(topicName1, 1000, &MainWindow::poseCallback, this);
}

void MainWindow::clickDo() {

    //获取desX,desY的值
    desX = lineX->text().toDouble();
    desY = lineY->text().toDouble();
    kp = linekp->text().toDouble();
    ki = lineki->text().toDouble();
    kd = linekd->text().toDouble();
    //测试角度
//    double jiao = calcuAngular(srcX, srcY, desX, desY);
//    ROS_INFO_STREAM(to_string(jiao*180/M_PI));

    new std::thread(&MainWindow::moveTurtle,this);
}


void MainWindow::clickSweepV() {
    kp = linekp->text().toDouble();
    ki = lineki->text().toDouble();
    kd = linekd->text().toDouble();

    new std::thread(&MainWindow::sweepV,this);
}

void MainWindow::clickSweepH() {
    kp = linekp->text().toDouble();
    ki = lineki->text().toDouble();
    kd = linekd->text().toDouble();

    new std::thread(&MainWindow::sweepH,this);
}

void MainWindow::clickSweepR() {
    kp = linekp->text().toDouble();
    ki = lineki->text().toDouble();
    kd = linekd->text().toDouble();

    new std::thread(&MainWindow::sweepR,this);
}

void MainWindow::clickStop() {
    stopFlag = 1;
}

//竖直扫地
void MainWindow::sweepV(){
    for (int i = 1; i < 11; i+=2) {
        desX = i;
        desY = 1;
        int  a = moveTurtle();
        if (a == 1) {
            return;
        }
        desX = i;
        desY = 10;
        int b = moveTurtle();
        if (b == 1) {
            return;
        }
    }
}

//水平扫地
void MainWindow::sweepH() {
    for (int i = 1; i < 12; i+=2) {
        desX =1;
        desY = i;
        int a = moveTurtle();
        if (a == 1) {
            return;
        }
        desX = 10;
        desY = i;
        int b = moveTurtle();
        if (b == 1) {
            return;
        }
    }
}

//回形扫地
void MainWindow::sweepR(){
    int i = 1;
    int j = 1;
    for (; i < 6 ; i+=2,j+=2) {
        desX = i;
        desY =j;
        int a = moveTurtle();
        if (a == 1) {
            return;
        }
        desX = i;
        desY = 12 - j;
        int b = moveTurtle();
        if (b == 1) {
            return;
        }

        desX = 12 - i;
        desY = 12 -j;
        int c = moveTurtle();
        if (c == 1) {
            return;
        }

        desX = 12 - i;
        desY = j;
        int d = moveTurtle();
        if (d == 1) {
            return;
        }
    }
}

void MainWindow::onUpdate() {
    ros::spinOnce();
    update();
    if (!ros::ok()) {
        close();
    }
}

//回调函数
void MainWindow::poseCallback(const turtlesim::Pose::ConstPtr& message) {
    srcX = message->x;
    srcY = message->y;
    turtle_angular = message->theta;
    labelX->setText(QString::fromStdString(to_string(srcX)));
    labelY->setText(QString::fromStdString(to_string(srcY)));
    labelTheta->setText(QString::fromStdString(to_string(turtle_angular*180/M_PI)));

}

//加算距离
double MainWindow::calcuDistance(double srcX, double srcY, double desX, double desY){
    return sqrt(pow(srcX - desX, 2) + pow(srcY - desY,2));
}

//计算角度
double MainWindow::calcuAngular(double srcX, double srcY, double desX, double desY){
    // 计算实时角度= 原坐标和目标位置连线的角度- 乌龟的角度
    // turtle_angular [-2*pi,2*pi]
    // line_theta [-pi,pi]
    //连线的角度
    double line_theta = atan2((desY - srcY), (desX - srcX));
    //先改变turtle_angular 的区间 改为[-pi,pi]
    if (turtle_angular > M_PI) {
        turtle_angular -= 2*M_PI;
    } else if (turtle_angular < -M_PI) {
        turtle_angular += 2*M_PI;
    }
    double dest_theta = line_theta - turtle_angular;
    //改变结果的区间 从[-2*pi,2*pi]到[-pi,pi]
    if (dest_theta > M_PI) {
        dest_theta -= 2*M_PI;
    } else if (dest_theta < -M_PI) {
        dest_theta += 2*M_PI;
    }
    return dest_theta;
}

//移动乌龟
int MainWindow::moveTurtle(){
    //计算距离
    double linear_distance = calcuDistance(srcX, srcY, desX, desY);
    //TODO 写日志进行测试
//    ROS_INFO_STREAM("distance"<<distance);
    //设置运行时间
    double t = 5;
    double linear_vel = 0;
    double thetaTime = t/10;

//    ros::Rate rate(hz);
//    for (int i = 0; i < t*hz; ++i) {
//        geometry_msgs::Twist twist;
//       linear_vel = distance/t;
//   linear_vel = distance;
//        twist.linear.x = linear_vel;
//        twist.angular.z = 0;
//        publisher.publish(twist);
//        t -= 1 / hz;
//        rate.sleep();
//    }
    double hz = 20;
    //速度的误差
    double linear_vel_error = 0;
    //记录所有的错误值
    double  linearTotalError = 0;
    //记录上一次的误差值
    double  linearLastError =0;
    //根据实时速度进行计算，计算误差
    ros::Rate rate(hz);
    //TODO 时间太长？
    double startTime = ros::Time::now().toSec();
    while (abs(calcuDistance(srcX, srcY, desX, desY)) > 0.1) {
        //判断是否要停止运动
        if (stopFlag) {
            stopFlag = 0;
            return 1;
        }

        //计算距离
        linear_distance = calcuDistance(srcX, srcY, desX, desY);
        //预期的速度
        double linear_target_vel = linear_distance / t;
        //速度的误差
        linear_vel_error = linear_target_vel - linear_vel;
        linearTotalError += linear_vel_error;
        double linearDelteError = linear_vel_error - linearLastError;
        linearLastError = linear_vel_error;
        linear_vel += kp * linear_vel_error;
        linear_vel += ki * linearTotalError;


        //***********角度***************
        double angular = calcuAngular(srcX, srcY, desX, desY);
        double theta = angular / thetaTime;

        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel;
        twist.angular.z = theta;
        velpublisher.publish(twist);


        std_msgs::Float64 msg;
        msg.data = linear_vel;
        plotpublisher.publish(msg);

        t -= 1 / hz;
        rate.sleep();
        ROS_INFO_STREAM("距离："<<linear_distance);
        ROS_INFO_STREAM(linear_vel);
        ROS_INFO_STREAM("时间："<<t);
    }
    double endTime = ros::Time::now().toSec();
    ROS_INFO_STREAM("time:"<<(endTime-startTime));
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;
    velpublisher.publish(twist);
    return 0;

}


