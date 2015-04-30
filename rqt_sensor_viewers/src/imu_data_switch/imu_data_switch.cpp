#include <iostream>

//#include <QTimer>
#include <QHBoxLayout>

#include "imu_data_switch.h"


IMUDataSwitch::IMUDataSwitch(QWidget* parent)    : rviz::Panel(parent) {

    client_ = nh_.serviceClient<yozakura_msgs::IMUDataSwitch>("imu_switch");

    imu_switch_btn_ = new QRadioButton("IMU Switch");
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(imu_switch_btn_);
    setLayout( layout );

    connect(imu_switch_btn_, SIGNAL(toggled(bool)), this, SLOT(callService()));
}

void IMUDataSwitch::callService() {
    if(imu_switch_btn_->isChecked() == 1){
        srv_.request.imu_data_switch = true;
    } else {
        srv_.request.imu_data_switch = false;
    }
    client_.call(srv_);
}

void IMUDataSwitch::save(rviz::Config config) const {
        rviz::Panel::save(config);
}

void IMUDataSwitch::load(const rviz::Config& config) {
        rviz::Panel::load(config);
}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(IMUDataSwitch, rviz::Panel)
