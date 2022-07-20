#ifndef GEODETIC_GRID_H_
#define GEODETIC_GRID_H_

#include <memory>
#include <ros/ros.h>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
class GAZEBO_VISIBLE GeodeticGrid : public GUIPlugin
{
  Q_OBJECT

  /// \brief Spacing between the axis labels and the axis lines
 private:
  static float LABEL_LINE_SPACING;

 private:
  static float LOCAL_VALUE_PRECISION;

 private:
  static float GLOBAL_VALUE_PRECISION;

  /// \brief Constructor
 public:
  GeodeticGrid();

  /// \brief Destructor
 public:
  virtual ~GeodeticGrid();

  // Documentation inherited.
 public:
  virtual void Load(sdf::ElementPtr _sdf);

  /// \brief Pointer to sdf
 protected:
  sdf::ElementPtr sdf;

  /// \brief Pointer to scene
 protected:
  rendering::ScenePtr scene;

 protected:
  rendering::UserCameraPtr userCamera;

  /// \brief Pointer to scene manager
 protected:
  std::shared_ptr<Ogre::SceneManager> manager;

 private:
  std::shared_ptr<Ogre::SceneNode> xAxisNode;

 private:
  std::shared_ptr<Ogre::SceneNode> yAxisNode;

  /// \brief Display label value based on frame (local or global)
 private:
  std::string frame;

  /// \brief Local origin coordinate of the grid
 private:
  ignition::math::Vector3d localOrigin;

  /// \brief Global origin, used to construct the lat/lon grid
 private:
  ignition::math::Vector3d globalOrigin;

  /// \brief Global value, used to compute lat/lon value and display it on the grid
 private:
  ignition::math::Vector3d globalValue;

  /// \brief Grid dimension, etc: 100 X 100
 private:
  ignition::math::Vector3d dimension;

  /// \brief Grid unit spacing
 private:
  double spacing;

  /// \brief Grid label size
 private:
  double labelSize;

  /// \brief All the event connections
 private:
  std::vector<event::ConnectionPtr> connections;

  /// \brief Transport node used for communication.
 private:
  ros::NodeHandle rosNode;

  /// \brief A GPS Local-to-Global service client
 private:
  ros::ServiceClient local2GlobalClient;

  /// \brief A GPS Global-to-Local service client
 private:
  ros::ServiceClient global2LocalClient;

  /// \brief Draw the grid using local coordinate (x-y-z)
 private:
  void DrawLocalGrid();

  /// \brief Draw the grid using global coordinate (lat-lon-alt)
 private:
  void DrawGlobalGrid();

  /// \brief Write label value to the grid using local or global coordinate
 private:
  void WriteGridLabel(double _value, int _precision, Ogre::SceneNode *axisNode, ignition::math::Vector3d _position,
                      ignition::math::Color _color, const rendering::MovableText::HorizAlign &_hAlign,
                      const rendering::MovableText::VertAlign &_vAlign);

  /// \brief Update label position with camera position
 private:
  void UpdateLabelsPosition();

  /// \brief Transform local coordinate to global coordinate
 private:
  ignition::math::Vector3d TransformFromGlobalCoord(double latitude_deg, double longitude_deg, double altitude);

  /// \brief Transform global coordinate to local coordinate
 private:
  ignition::math::Vector3d TransformToGlobalCoord(double x, double y, double z);
};
}

#endif
