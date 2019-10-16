#include "ros/ros.h"
#include <iostream>
#include "MainWindow.h"
#include <QApplication>
using namespace std;

int main(int argc, char **argv) {
    string nodeName = "sweepRobot";
    ros::init(argc, argv, nodeName);
    ros::NodeHandle node;
    QApplication app(argc, argv);
    MainWindow window(node);

    window.show();

    return app.exec();
}