#include <sstream>
#include <gazebo_geodetic_plugins/vehicle_position_widget.h>
#include <uuv_world_ros_plugins_msgs/TransformToSphericalCoord.h>

namespace gazebo
{
VehiclePositionWidget::VehiclePositionWidget() : GUIPlugin()
{
  // Set the widget background color
  this->setStyleSheet("background-color : rgba(100, 100, 100, 255)");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create coordinate label
  QLabel *gpsCoordinate = new QLabel(tr("Latitude: 0.0000°,  Longitude: 0.0000°,  Altitude: 00.00 m"));

  // Set the label text color
  gpsCoordinate->setStyleSheet("color : white");

  connect(this, SIGNAL(setPositionText(QString)), gpsCoordinate, SLOT(setText(QString)), Qt::QueuedConnection);

  // Add the label to the main layout
  mainLayout->addWidget(gpsCoordinate);

  // Add margins to main layout for clearer view
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Add the main layout to the widget
  this->setLayout(mainLayout);

  // Position this widget to top-left corner
  this->move(10, 10);

  // Call service that transform the Cartesian to lat/lot/alt info
  this->gpsClient = this->rosNode.serviceClient<uuv_world_ros_plugins_msgs::TransformToSphericalCoord>(
      "/gazebo/transform_to_spherical_coordinates");

  // Subscribe to get the UUV position in Cartesian
  this->rosSub = this->rosNode.subscribe("rexrov/pose_gt", 1000, &VehiclePositionWidget::onSubscribePosition, this);
}

VehiclePositionWidget::~VehiclePositionWidget()
{
  // Clear rosNode and its service clients
  this->rosNode.shutdown();
  this->gpsClient.shutdown();
  this->rosSub.shutdown();
}

void VehiclePositionWidget::onSubscribePosition(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::ostringstream stream;
  uuv_world_ros_plugins_msgs::TransformToSphericalCoord srv;

  srv.request.input.x = msg->pose.pose.position.x;
  srv.request.input.y = msg->pose.pose.position.y;
  srv.request.input.z = msg->pose.pose.position.z;

  if (this->gpsClient.call(srv))
  {
    stream.str("");

    // Latitude with the precision of 5 decimal places, ex: 0.00001° -> 1.1112m
    stream << "Latitude: " << std::fixed << std::setprecision(5) << srv.response.latitude_deg << "°,  ";

    // Longitude with the precision of 5 decimal places, ex: 0.00001° -> 1.1112m
    stream << "Longitude: " << std::fixed << std::setprecision(5) << srv.response.longitude_deg << "°,  ";

    // Altitude with the precision of 2 decimal places
    stream << "Altitude: " << std::fixed << std::setprecision(2) << srv.response.altitude << " m";

    // Update a new coordinate label
    this->setPositionText(QString::fromStdString(stream.str()));

    // Move and resize the widget
    this->move(10, 10);
    this->adjustSize();
  }
  else
  {
    ROS_ERROR_DELAYED_THROTTLE(60, "Failed to call service /gazebo/transform_to_spherical_coordinates");
  }
}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(VehiclePositionWidget)
}