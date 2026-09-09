#ifndef TEST_C__LINE_DRAW_TOOL_HPP_
#define TEST_C__LINE_DRAW_TOOL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreVector3.h>

#include <QCursor>
#include <QLabel>
#include <QObject>
#include <QPushButton>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rviz_common
{
class PanelDockWidget;
namespace properties
{
class BoolProperty;
class FloatProperty;
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace test_c
{

class LineDrawTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  LineDrawTool();
  ~LineDrawTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private Q_SLOTS:
  void updateMarkerTopic();
  void updatePathTopic();
  void updateClickedPointsTopic();
  void updateLaneTopic();
  void updateLineWidth();
  void updateCornerRadius();

private:
  void addClickedPoint(const Ogre::Vector3 & position);
  void publishAll();
  void publishClickedPoints();
  void publishMarkers();
  void clearMarkers();
  void recreateMarkerPublisher();
  void recreateClickedPointsPublisher();
  void handleCommand(const std_msgs::msg::UInt8::SharedPtr msg);
  void setDrawingEnabled(bool enabled);
  void publishStatus(const std::string & text);
  void createControlPane();
  void updateControlPane();
  void clearAll();
  void selectThisTool();

  std::vector<Point2D> roundedPath();
  std::vector<Point2D> interpolatedPath(const std::vector<Point2D> & path_points) const;
  nav_msgs::msg::Path buildPathMsg(const std::vector<Point2D> & path_points) const;
  double yawAt(const std::vector<Point2D> & path_points, std::size_t index) const;
  static double normalizeAngle(double angle);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr request_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr clicked_points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr command_sub_;
  std::unique_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

  rviz_common::properties::StringProperty * marker_topic_property_{nullptr};
  rviz_common::properties::StringProperty * path_topic_property_{nullptr};
  rviz_common::properties::StringProperty * clicked_points_topic_property_{nullptr};
  rviz_common::properties::StringProperty * lane_topic_property_{nullptr};
  rviz_common::properties::FloatProperty * line_width_property_{nullptr};
  rviz_common::properties::FloatProperty * corner_radius_property_{nullptr};
  rviz_common::properties::FloatProperty * sample_step_property_{nullptr};

  std::vector<Point2D> clicked_points_;
  bool active_{false};
  bool drawing_enabled_{false};
  double minimum_corner_radius_m_{0.0};
  std::string path_generation_error_;
  int last_marker_count_{0};

  rviz_common::PanelDockWidget * control_dock_{nullptr};
  QLabel * point_count_label_{nullptr};
  QLabel * mode_label_{nullptr};
  QLabel * status_label_{nullptr};
  QPushButton * draw_button_{nullptr};
  QPushButton * publish_button_{nullptr};
  QPushButton * clear_button_{nullptr};
};

}  // namespace test_c

#endif  // TEST_C__LINE_DRAW_TOOL_HPP_
