#include <sstream>
#include <gazebo_geodetic_plugins/geodetic_grid.h>
#include <uuv_world_ros_plugins_msgs/TransformFromSphericalCoord.h>
#include <uuv_world_ros_plugins_msgs/TransformToSphericalCoord.h>
#include <uuv_gazebo_plugins/Def.hh>  // Use Str2Vector function from this dependency

namespace gazebo
{
float GeodeticGrid::LABEL_LINE_SPACING(0.06);
float GeodeticGrid::LOCAL_VALUE_PRECISION(1);
float GeodeticGrid::GLOBAL_VALUE_PRECISION(5);

GeodeticGrid::GeodeticGrid() : frame("global"), localOrigin(0, 0, 0), dimension(100, 100, 0), spacing(1), labelSize(0.2)
{
  this->move(10, 10);
  this->resize(0, 0);

  // Utilize local-to-global transformation service
  this->local2GlobalClient = this->rosNode.serviceClient<uuv_world_ros_plugins_msgs::TransformToSphericalCoord>(
      "/gazebo/transform_to_spherical_coordinates");

  // Utilize global-to-local transformation service
  this->global2LocalClient = this->rosNode.serviceClient<uuv_world_ros_plugins_msgs::TransformFromSphericalCoord>(
      "/gazebo/transform_from_spherical_coordinates");
}

GeodeticGrid::~GeodeticGrid()
{
  // Clear connection events
  this->connections.clear();

  // Clear rosNode and its service clients
  this->rosNode.shutdown();
  this->local2GlobalClient.shutdown();
  this->global2LocalClient.shutdown();

  manager.reset();
  xAxisNode.reset();
  yAxisNode.reset();

  // Scene deletion handled by ogre which automatically removes manager and other nodes
  this->scene = NULL;

  // Reset sdf, this is how gazebo implements the reset
  this->sdf->Reset();
  this->sdf.reset();
}

void GeodeticGrid::Load(sdf::ElementPtr _sdf)
{
  this->scene = rendering::get_scene();
  GZ_ASSERT(this->scene != NULL, "Invalid scene pointer");

  this->userCamera = this->scene->GetUserCamera(0);
  GZ_ASSERT(this->userCamera != NULL, "No user_camera is available");

  this->manager.reset(this->scene->OgreSceneManager());

  this->sdf = _sdf;

  // Custom frame-based label that grid will display
  if (this->sdf->HasElement("frame"))
  {
    // Local or Global frame
    this->frame = this->sdf->GetElement("frame")->GetValue()->GetAsString();
    GZ_ASSERT(this->frame == "local" || this->frame == "global", "The frame must be either local or global");
  }

  // Custom origin coordinate provided by user
  if (this->sdf->HasElement("origin"))
  {
    sdf::ElementPtr originElement = this->sdf->GetElement("origin");

    // Get x-y-z values from the xyz attribute
    if (originElement->HasAttribute("xyz"))
    {
      std::vector<double> xyz = Str2Vector(originElement->GetAttribute("xyz")->GetAsString());

      // Local origin
      this->localOrigin.X(xyz[0]);
      this->localOrigin.Y(xyz[1]);
      this->localOrigin.Z(xyz[2]);

      // Global origin, used to construct the lat/lon grid
      this->globalOrigin = this->TransformToGlobalCoord(-xyz[0], -xyz[1], xyz[2]);

      // Global value, used to compute lat/lon value and display it on the grid
      this->globalValue = this->TransformToGlobalCoord(xyz[0], xyz[1], xyz[2]);
    }
  }

  // Custom grid dimension provided by user
  if (this->sdf->HasElement("dimension"))
  {
    std::vector<double> dimension = Str2Vector(this->sdf->GetElement("dimension")->GetValue()->GetAsString());

    // Accept whole number dimension only
    int dimX = dimension[0];
    int dimY = dimension[1];

    // Convert odd dimension to even dimension for easy computation
    this->dimension.X((dimX % 2 == 0) ? dimX : dimX + 1);
    this->dimension.Y((dimY % 2 == 0) ? dimY : dimY + 1);
  }

  // Custom grid spacing provided by user
  if (this->sdf->HasElement("spacing"))
  {
    this->spacing = this->sdf->Get<double>("spacing");
  }

  // Custom grid label size provided by user
  if (this->sdf->HasElement("label_size"))
  {
    this->labelSize = this->sdf->Get<double>("label_size");
  }

  // Draw grid in local or global coordinate
  if (this->frame == "local")
  {
    this->DrawLocalGrid();
  }
  else
  {
    this->DrawGlobalGrid();
  }

  // Callback called on pre-render
  this->connections.push_back(event::Events::ConnectPreRender(std::bind(&GeodeticGrid::UpdateLabelsPosition, this)));
}

void GeodeticGrid::WriteGridLabel(double _value, int _precision, Ogre::SceneNode *axisNode,
                                  ignition::math::Vector3d _position, ignition::math::Color _color,
                                  const rendering::MovableText::HorizAlign &_hAlign,
                                  const rendering::MovableText::VertAlign &_vAlign)
{
  // Set precision of the label value (1 decimal for local and 5 decimal for global)
  std::ostringstream valueText;
  valueText.str("");
  valueText << std::fixed << std::setprecision(_precision) << _value;

  // Create x label text
  auto labelText = new rendering::MovableText;

  // Display value label text with Red color
  labelText->Load("label", valueText.str(), "Arial", this->labelSize, _color);

  // Center horizontally and display value label text above the node
  labelText->SetTextAlignment(_hAlign, _vAlign);
  labelText->SetShowOnTop(true);

  // Create value label node
  Ogre::SceneNode *labelNode = axisNode->createChildSceneNode();

  // Attach value label text to the value label node
  labelNode->attachObject(labelText);

  labelNode->setPosition(_position.X(), _position.Y(), _position.Z());
}

void GeodeticGrid::DrawLocalGrid()
{
  double x_offset = this->dimension.X() / 2.0;
  double y_offset = this->dimension.Y() / 2.0;

  // Create x-axis
  this->xAxisNode.reset(this->manager->getRootSceneNode()->createChildSceneNode("x_axis"));
  Ogre::ManualObject *xAxisObject = this->manager->createManualObject("x_axis");

  this->xAxisNode->setVisible(true);
  xAxisObject->setVisible(true);
  xAxisObject->setRenderQueueGroup(1);

  xAxisObject->clear();
  xAxisObject->begin("Gazebo/Red", Ogre::RenderOperation::OT_LINE_LIST);

  // Draw x lines
  for (int i = 0; i <= (x_offset / this->spacing); i++)
  {
    double x = i * this->spacing;
    // Draw positive x line
    xAxisObject->position(this->localOrigin.X() + x, this->localOrigin.Y() - y_offset, this->localOrigin.Z());
    xAxisObject->position(this->localOrigin.X() + x, this->localOrigin.Y() + y_offset, this->localOrigin.Z());

    // Write positive x label
    this->WriteGridLabel(this->localOrigin.X() + x, LOCAL_VALUE_PRECISION, this->xAxisNode.get(),
                         ignition::math::Vector3d(this->localOrigin.X() + x, this->localOrigin.Y(),
                                                  this->localOrigin.Z() - GeodeticGrid::LABEL_LINE_SPACING),
                         ignition::math::Color::Red, rendering::MovableText::H_CENTER, rendering::MovableText::V_ABOVE);

    // Prevent -0.0 case
    if (x > 0)
    {
      // Draw negative x line
      xAxisObject->position(this->localOrigin.X() - x, this->localOrigin.Y() - y_offset, this->localOrigin.Z());
      xAxisObject->position(this->localOrigin.X() - x, this->localOrigin.Y() + y_offset, this->localOrigin.Z());

      // Write negative x label
      this->WriteGridLabel(this->localOrigin.X() - x, LOCAL_VALUE_PRECISION, this->xAxisNode.get(),
                           ignition::math::Vector3d(this->localOrigin.X() - x, this->localOrigin.Y(),
                                                    this->localOrigin.Z() - GeodeticGrid::LABEL_LINE_SPACING),
                           ignition::math::Color::Red, rendering::MovableText::H_CENTER,
                           rendering::MovableText::V_ABOVE);
    }
  }

  xAxisObject->end();
  this->xAxisNode->attachObject(xAxisObject);

  // Create y-axis
  this->yAxisNode.reset(this->manager->getRootSceneNode()->createChildSceneNode("y_axis"));
  Ogre::ManualObject *yAxisObject = this->manager->createManualObject("y_axis");

  this->yAxisNode->setVisible(true);
  yAxisObject->setVisible(true);
  yAxisObject->setRenderQueueGroup(1);

  yAxisObject->clear();
  yAxisObject->begin("Gazebo/Green", Ogre::RenderOperation::OT_LINE_LIST);

  // Draw y lines
  for (int i = 0; i <= (y_offset / this->spacing); i++)
  {
    double y = i * this->spacing;
    // Draw positive y line
    yAxisObject->position(this->localOrigin.X() - x_offset, this->localOrigin.Y() + y, this->localOrigin.Z());
    yAxisObject->position(this->localOrigin.X() + x_offset, this->localOrigin.Y() + y, this->localOrigin.Z());

    // Write positive y label
    this->WriteGridLabel(this->localOrigin.Y() + y, LOCAL_VALUE_PRECISION, this->yAxisNode.get(),
                         ignition::math::Vector3d(this->localOrigin.X(), this->localOrigin.Y() + y,
                                                  this->localOrigin.Z() - GeodeticGrid::LABEL_LINE_SPACING),
                         ignition::math::Color::Green, rendering::MovableText::H_CENTER,
                         rendering::MovableText::V_BELOW);

    // Prevent -0.0 case
    if (y > 0)
    {
      // Draw negative y line
      yAxisObject->position(this->localOrigin.X() - x_offset, this->localOrigin.Y() - y, this->localOrigin.Z());
      yAxisObject->position(this->localOrigin.X() + x_offset, this->localOrigin.Y() - y, this->localOrigin.Z());

      // Write negative y label
      this->WriteGridLabel(this->localOrigin.Y() - y, LOCAL_VALUE_PRECISION, this->yAxisNode.get(),
                           ignition::math::Vector3d(this->localOrigin.X(), this->localOrigin.Y() - y,
                                                    this->localOrigin.Z() - GeodeticGrid::LABEL_LINE_SPACING),
                           ignition::math::Color::Green, rendering::MovableText::H_CENTER,
                           rendering::MovableText::V_BELOW);
    }
  }

  yAxisObject->end();
  this->yAxisNode->attachObject(yAxisObject);
}

void GeodeticGrid::DrawGlobalGrid()
{
  // Get current position of the user camera
  ignition::math::Vector3d cameraPosition = this->userCamera->WorldPosition();

  // Get min/max longitude range from the origin
  double minLonRange = this->globalOrigin.Y() - (this->dimension.Y() / 2) * this->spacing;
  double maxLonRange = this->globalOrigin.Y() + (this->dimension.Y() / 2) * this->spacing;

  // Get min/max latitude range from the origin
  double minLatRange = this->globalOrigin.X() - (this->dimension.X() / 2) * this->spacing;
  double maxLatRange = this->globalOrigin.X() + (this->dimension.X() / 2) * this->spacing;

  // Get the max value of the longitude and latitude
  double maxLonValue = this->globalValue.Y() + (this->dimension.Y() / 2) * this->spacing;
  double maxLatValue = this->globalValue.X() + (this->dimension.X() / 2) * this->spacing;

  // Create longitude axis
  this->xAxisNode.reset(this->manager->getRootSceneNode()->createChildSceneNode("lon_axis"));
  Ogre::ManualObject *xAxisObject = this->manager->createManualObject("lon_axis");

  xAxisNode->setVisible(true);
  xAxisObject->setVisible(true);
  xAxisObject->setRenderQueueGroup(1);

  xAxisObject->clear();
  xAxisObject->begin("Gazebo/Red", Ogre::RenderOperation::OT_LINE_LIST);

  // Draw longitude lines
  for (int lonNum = 0; lonNum <= dimension.Y(); lonNum++)
  {
    double lon = minLonRange + lonNum * this->spacing;

    ignition::math::Vector3d begin = this->TransformFromGlobalCoord(minLatRange, lon, this->globalOrigin.Z());
    ignition::math::Vector3d end = this->TransformFromGlobalCoord(maxLatRange, lon, this->globalOrigin.Z());

    xAxisObject->position(begin.X(), begin.Y(), begin.Z());
    xAxisObject->position(end.X(), end.Y(), end.Z());

    // Write longitude values
    this->WriteGridLabel(maxLonValue - lonNum * this->spacing, GLOBAL_VALUE_PRECISION, this->xAxisNode.get(),
                         ignition::math::Vector3d(begin.X(), cameraPosition.Y(),
                                                  this->localOrigin.Z() - GeodeticGrid::LABEL_LINE_SPACING),
                         ignition::math::Color::Red, rendering::MovableText::H_CENTER, rendering::MovableText::V_ABOVE);
  }

  xAxisObject->end();
  xAxisNode->attachObject(xAxisObject);

  // Create latitude axis
  this->yAxisNode.reset(this->manager->getRootSceneNode()->createChildSceneNode("lat_axis"));
  Ogre::ManualObject *yAxisObject = this->manager->createManualObject("lat_axis");

  yAxisNode->setVisible(true);
  yAxisObject->setVisible(true);
  yAxisObject->setRenderQueueGroup(1);

  yAxisObject->clear();
  yAxisObject->begin("Gazebo/Green", Ogre::RenderOperation::OT_LINE_LIST);

  // Draw latitude lines
  for (int latNum = 0; latNum <= dimension.X(); latNum++)
  {
    double lat = minLatRange + latNum * this->spacing;

    ignition::math::Vector3d begin = this->TransformFromGlobalCoord(lat, minLonRange, this->globalOrigin.Z());
    ignition::math::Vector3d end = this->TransformFromGlobalCoord(lat, maxLonRange, this->globalOrigin.Z());

    yAxisObject->position(begin.X(), begin.Y(), begin.Z());
    yAxisObject->position(end.X(), end.Y(), end.Z());

    // Write latitude values
    this->WriteGridLabel(maxLatValue - latNum * this->spacing, GLOBAL_VALUE_PRECISION, this->yAxisNode.get(),
                         ignition::math::Vector3d(cameraPosition.X(), begin.Y(),
                                                  this->localOrigin.Z() - GeodeticGrid::LABEL_LINE_SPACING),
                         ignition::math::Color::Green, rendering::MovableText::H_CENTER,
                         rendering::MovableText::V_BELOW);
  }

  yAxisObject->end();
  yAxisNode->attachObject(yAxisObject);
}

void GeodeticGrid::UpdateLabelsPosition()
{
  // Get current position of the user camera
  ignition::math::Vector3d cameraPosition = this->userCamera->WorldPosition();

  // Get a list of x and y labels
  Ogre::SceneNode::ChildNodeIterator xLabels = this->xAxisNode->getChildIterator();
  Ogre::SceneNode::ChildNodeIterator yLabels = this->yAxisNode->getChildIterator();

  // Iterate each x label and update the position
  while (xLabels.hasMoreElements())
  {
    Ogre::SceneNode *current = dynamic_cast<Ogre::SceneNode *>(xLabels.getNext());

    current->setPosition(current->getPosition().x, cameraPosition.Y(), current->getPosition().z);

    current->_update(true, false);
  }

  // Iterate each y label and update the position
  while (yLabels.hasMoreElements())
  {
    Ogre::SceneNode *current = dynamic_cast<Ogre::SceneNode *>(yLabels.getNext());

    current->setPosition(cameraPosition.X(), current->getPosition().y, current->getPosition().z);

    current->_update(true, false);
  }
}

ignition::math::Vector3d GeodeticGrid::TransformFromGlobalCoord(double latitude_deg, double longitude_deg,
                                                                double altitude)
{
  uuv_world_ros_plugins_msgs::TransformFromSphericalCoord srv;
  srv.request.latitude_deg = latitude_deg;
  srv.request.longitude_deg = longitude_deg;
  srv.request.altitude = altitude;

  if (this->global2LocalClient.call(srv))
  {
    return ignition::math::Vector3d(srv.response.output.x, srv.response.output.y, srv.response.output.z);
  }
  return ignition::math::Vector3d::Zero;
}

ignition::math::Vector3d GeodeticGrid::TransformToGlobalCoord(double x, double y, double z)
{
  uuv_world_ros_plugins_msgs::TransformToSphericalCoord srv;
  srv.request.input.x = x;
  srv.request.input.y = y;
  srv.request.input.z = z;

  if (this->local2GlobalClient.call(srv))
  {
    return ignition::math::Vector3d(srv.response.latitude_deg, srv.response.longitude_deg, srv.response.altitude);
  }
  return ignition::math::Vector3d::Zero;
}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GeodeticGrid)
}