#ifndef IMU_DATA_SWITCH_H
#define IMU_DATA_SWITCH_H

#include <ros/ros.h>
#include "yozakura_msgs/IMUDataSwitch.h"

#include <QWidget>
#include <QRadioButton>
#include <rviz/panel.h>

class QRadioButton;

class IMUDataSwitch: public rviz::Panel {
    Q_OBJECT
public:
    IMUDataSwitch(QWidget* parent = 0);

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    void callService();

protected:
    QRadioButton* imu_switch_btn_;
    yozakura_msgs::IMUDataSwitch srv_;
    ros::ServiceClient client_;
    ros::NodeHandle nh_;
};


#endif
