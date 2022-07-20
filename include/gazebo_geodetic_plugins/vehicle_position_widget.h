#ifndef VEHICLE_POSITION_WIDGET_H_
#define VEHICLE_POSITION_WIDGET_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

namespace gazebo
{
class GAZEBO_VISIBLE VehiclePositionWidget : public GUIPlugin
{
  Q_OBJECT

  /// \brief Constructor
 public:
  VehiclePositionWidget();

  /// \brief Destructor
 public:
  virtual ~VehiclePositionWidget();

  /// \brief A signal used to set the vehical position text box.
  /// \param[in] _string String representation of vehical position in spherical coordinate.
 signals:
  void setPositionText(QString _string);

  /// \brief Callback that received vehicle position messages.
  /// \param[in] _msg Vehicle position that is received in Odometry type.
 protected:
  void onSubscribePosition(const nav_msgs::Odometry::ConstPtr &msg);

  /// \brief A node use for ROS transport
 private:
  ros::NodeHandle rosNode;

  /// \brief A ROS subscriber
 private:
  ros::Subscriber rosSub;

  /// \brief A GPS Service Client
 private:
  ros::ServiceClient gpsClient;
};
}

#endif
