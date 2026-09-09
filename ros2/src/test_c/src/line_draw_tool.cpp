// NVIDIA/VTK环境中的TinyXML2符号重命名宏可能经Ogre/RViz头文件泄漏。
// 必须先加载pluginlib及系统TinyXML2，避免XMLDocument被改成XMLTDocument。
#include "pluginlib/class_loader.hpp"

#include "test_c/line_draw_tool.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QFont>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/panel_dock_widget.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/window_manager_interface.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr uint8_t kCmdDraw = 1;
constexpr uint8_t kCmdPublish = 2;
constexpr uint8_t kCmdClear = 3;

double clamp(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

double distance(const test_c::LineDrawTool::Point2D & a, const test_c::LineDrawTool::Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

namespace test_c
{

LineDrawTool::LineDrawTool()
{
  shortcut_key_ = 'p';

  marker_topic_property_ = new rviz_common::properties::StringProperty(
    "Marker Topic", "/test_c/click_path_markers",
    "显示连续打点和倒角路径的 MarkerArray 话题。",
    getPropertyContainer(), SLOT(updateMarkerTopic()), this);

  path_topic_property_ = new rviz_common::properties::StringProperty(
    "Request Path Topic", "/test_c/click_path_request",
    "发送给 pathborn.py 的 nav_msgs/Path 请求话题；pathborn生成Lane后交给state_path。",
    getPropertyContainer(), SLOT(updatePathTopic()), this);

  clicked_points_topic_property_ = new rviz_common::properties::StringProperty(
    "Clicked Points Topic", "/test_c/click_points_request",
    "只发布原始点击点的 nav_msgs/Path 话题，不做倒角和插值。",
    getPropertyContainer(), SLOT(updateClickedPointsTopic()), this);

  line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line Width", 0.18,
    "RViz 线宽，单位 m。",
    getPropertyContainer(), SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.01);

  corner_radius_property_ = new rviz_common::properties::FloatProperty(
    "Corner Radius", 1.5,
    "连续打点路径的倒角圆半径，单位 m。",
    getPropertyContainer(), SLOT(updateCornerRadius()), this);
  corner_radius_property_->setMin(0.0);

  sample_step_property_ = new rviz_common::properties::FloatProperty(
    "Sample Step", 0.3,
    "倒角圆弧采样间距，单位 m。",
    getPropertyContainer(), SLOT(updateCornerRadius()), this);
  sample_step_property_->setMin(0.05);


}

void LineDrawTool::onInitialize()
{
  setName("Click Path Tool");
  setDescription("Left-click continuous points on map. Right-click or Publish Path publishes Path.");
  setCursor(QCursor(Qt::CrossCursor));

  projection_finder_ = std::make_unique<rviz_rendering::ViewportProjectionFinder>();
  auto node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    setStatus("No RViz ROS node.");
    return;
  }
  raw_node_ = node_abstraction->get_raw_node();
  double configured_corner_radius = corner_radius_property_->getFloat();
  if (raw_node_->has_parameter("test_c_corner_radius_m")) {
    raw_node_->get_parameter("test_c_corner_radius_m", configured_corner_radius);
  } else {
    configured_corner_radius = raw_node_->declare_parameter<double>(
      "test_c_corner_radius_m", configured_corner_radius);
  }
  if (raw_node_->has_parameter("test_c_min_corner_radius_m")) {
    raw_node_->get_parameter("test_c_min_corner_radius_m", minimum_corner_radius_m_);
  } else {
    minimum_corner_radius_m_ = raw_node_->declare_parameter<double>(
      "test_c_min_corner_radius_m", 0.0);
  }
  minimum_corner_radius_m_ = std::max(0.0, minimum_corner_radius_m_);
  corner_radius_property_->setMin(minimum_corner_radius_m_);
  corner_radius_property_->setFloat(
    std::max(configured_corner_radius, minimum_corner_radius_m_));
  status_pub_ = raw_node_->create_publisher<std_msgs::msg::String>(
    "/test_c/line_draw/status", rclcpp::QoS(10).transient_local());
  command_sub_ = raw_node_->create_subscription<std_msgs::msg::UInt8>(
    "/test_c/line_draw/command", rclcpp::QoS(10),
    [this](const std_msgs::msg::UInt8::SharedPtr msg) {
      handleCommand(msg);
    });
  recreateMarkerPublisher();
  request_pub_ = raw_node_->create_publisher<nav_msgs::msg::Path>(
    path_topic_property_->getStdString(), rclcpp::QoS(1).reliable().transient_local());
  recreateClickedPointsPublisher();
  createControlPane();
  publishMarkers();
  publishClickedPoints();
  publishStatus(
    "LineDrawTool 已加载，当前最小转弯半径=" +
    std::to_string(corner_radius_property_->getFloat()) + " m。");
}

void LineDrawTool::activate()
{
  active_ = true;
  createControlPane();
  if (control_dock_) {
    control_dock_->show();
    control_dock_->raise();
  }
  setStatus("Use right panel: 打点 / 发布 / 清除.");
}

void LineDrawTool::deactivate()
{
  active_ = false;
}

int LineDrawTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!active_ || !projection_finder_ || context_->getFixedFrame() != "map") {
    return Render;
  }

  if (event.rightDown()) {
    publishAll();
    return Render;
  }

  if (!event.leftDown()) {
    auto * view_manager = context_ ? context_->getViewManager() : nullptr;
    auto * view_controller = view_manager ? view_manager->getCurrent() : nullptr;
    if (view_controller) {
      view_controller->handleMouseEvent(event);
    }
    return Render;
  }

  if (!drawing_enabled_) {
    publishStatus("还没有进入打点模式：请先点击右侧 Panel 的“打点”。");
    return Render;
  }

  const auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  if (!projection.first) {
    setStatus("Mouse ray did not hit map XY plane.");
    return Render;
  }

  addClickedPoint(projection.second);
  return Render;
}

void LineDrawTool::updateMarkerTopic()
{
  recreateMarkerPublisher();
  publishMarkers();
}

void LineDrawTool::updatePathTopic()
{
  if (!raw_node_) {
    return;
  }
  request_pub_ = raw_node_->create_publisher<nav_msgs::msg::Path>(
    path_topic_property_->getStdString(), rclcpp::QoS(1).reliable().transient_local());
}

void LineDrawTool::updateClickedPointsTopic()
{
  recreateClickedPointsPublisher();
  publishClickedPoints();
}

void LineDrawTool::updateLaneTopic()
{
}

void LineDrawTool::updateLineWidth()
{
  publishMarkers();
}

void LineDrawTool::updateCornerRadius()
{
}

void LineDrawTool::addClickedPoint(const Ogre::Vector3 & position)
{
  clicked_points_.push_back({position.x, position.y});
  publishMarkers();
  publishClickedPoints();
  updateControlPane();
  publishStatus("已添加点，当前点数=" + std::to_string(clicked_points_.size()));
}

void LineDrawTool::publishAll()
{
  publishClickedPoints();
  const auto rounded_points = roundedPath();
  if (!path_generation_error_.empty()) {
    publishStatus("发布失败：" + path_generation_error_);
    return;
  }
  const auto path_points = interpolatedPath(rounded_points);
  if (path_points.size() < 2) {
    publishStatus("发布失败：至少需要 2 个点。");
    return;
  }
  if (request_pub_) {
    request_pub_->publish(buildPathMsg(path_points));
  }
  publishMarkers();
  updateControlPane();
  publishStatus("已发布路径，等待控制器接管。点数=" + std::to_string(path_points.size()));
  if (raw_node_) {
    RCLCPP_INFO(raw_node_->get_logger(), "ClickPathTool published request path_points=%zu",
    path_points.size());
  }
}

void LineDrawTool::publishClickedPoints()
{
  if (!clicked_points_pub_) {
    return;
  }
  clicked_points_pub_->publish(buildPathMsg(clicked_points_));
}

std::vector<LineDrawTool::Point2D> LineDrawTool::roundedPath()
{
  path_generation_error_.clear();
  if (clicked_points_.size() < 3 || corner_radius_property_->getFloat() <= 1.0e-6) {
    return clicked_points_;
  }

  const double radius = corner_radius_property_->getFloat();
  const double sample_step = std::max(0.05, static_cast<double>(sample_step_property_->getFloat()));
  std::vector<Point2D> output;
  output.push_back(clicked_points_.front());

  for (std::size_t i = 1; i + 1 < clicked_points_.size(); ++i) {
    const Point2D & prev = clicked_points_[i - 1];
    const Point2D & cur = clicked_points_[i];
    const Point2D & next = clicked_points_[i + 1];
    const double len1 = distance(prev, cur);
    const double len2 = distance(cur, next);
    if (len1 < 1.0e-6 || len2 < 1.0e-6) {
      output.push_back(cur);
      continue;
    }

    const double v1x = (cur.x - prev.x) / len1;
    const double v1y = (cur.y - prev.y) / len1;
    const double v2x = (next.x - cur.x) / len2;
    const double v2y = (next.y - cur.y) / len2;
    const double alpha = std::acos(clamp(v1x * v2x + v1y * v2y, -1.0, 1.0));
    if (alpha < 1.0e-3 || std::abs(kPi - alpha) < 1.0e-3) {
      output.push_back(cur);
      continue;
    }

    const double trim_requested = radius * std::tan(alpha * 0.5);
    const double available_trim = std::min(len1 * 0.45, len2 * 0.45);
    if (minimum_corner_radius_m_ > 0.0 && trim_requested > available_trim + 1.0e-6) {
      const double available_radius = available_trim / std::tan(alpha * 0.5);
      path_generation_error_ =
        "第" + std::to_string(i + 1) + "个转角相邻点距离太近，只能形成约" +
        std::to_string(available_radius) + " m圆角，小于当前农具要求的" +
        std::to_string(minimum_corner_radius_m_) + " m；请把相邻点击点拉远。";
      return {};
    }
    const double trim = std::min(trim_requested, available_trim);
    if (trim < 1.0e-4) {
      output.push_back(cur);
      continue;
    }

    const Point2D arc_start{cur.x - v1x * trim, cur.y - v1y * trim};
    const Point2D arc_end{cur.x + v2x * trim, cur.y + v2y * trim};
    const double actual_radius = trim / std::tan(alpha * 0.5);
    double bisx = -v1x + v2x;
    double bisy = -v1y + v2y;
    const double bis_len = std::hypot(bisx, bisy);
    if (bis_len < 1.0e-6) {
      output.push_back(cur);
      continue;
    }
    bisx /= bis_len;
    bisy /= bis_len;
    const double center_distance = actual_radius / std::cos(alpha * 0.5);
    const Point2D center{cur.x + bisx * center_distance, cur.y + bisy * center_distance};

    output.push_back(arc_start);
    double a0 = std::atan2(arc_start.y - center.y, arc_start.x - center.x);
    double a1 = std::atan2(arc_end.y - center.y, arc_end.x - center.x);
    const bool left_turn = (v1x * v2y - v1y * v2x) > 0.0;
    if (left_turn && a1 < a0) {
      a1 += 2.0 * kPi;
    } else if (!left_turn && a1 > a0) {
      a1 -= 2.0 * kPi;
    }
    const double arc_len = std::abs(a1 - a0) * actual_radius;
    const int steps = std::max(2, static_cast<int>(std::ceil(arc_len / sample_step)));
    for (int step = 1; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      const double a = a0 + (a1 - a0) * t;
      output.push_back({center.x + actual_radius * std::cos(a), center.y + actual_radius * std::sin(a)});
    }
  }

  output.push_back(clicked_points_.back());
  return output;
}

std::vector<LineDrawTool::Point2D> LineDrawTool::interpolatedPath(
  const std::vector<Point2D> & path_points) const
{
  if (path_points.size() < 2) {
    return path_points;
  }

  const double step = std::max(0.05, static_cast<double>(sample_step_property_->getFloat()));
  std::vector<Point2D> output;
  output.reserve(path_points.size() * 2);
  output.push_back(path_points.front());

  for (std::size_t i = 1; i < path_points.size(); ++i) {
    const auto & a = path_points[i - 1];
    const auto & b = path_points[i];
    const double len = distance(a, b);
    if (len < 1.0e-6) {
      continue;
    }
    const int parts = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int k = 1; k <= parts; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(parts);
      output.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    }
  }
  return output;
}

nav_msgs::msg::Path LineDrawTool::buildPathMsg(const std::vector<Point2D> & path_points) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = raw_node_->now();
  path.header.frame_id = "map";
  for (std::size_t i = 0; i < path_points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = path_points[i].x;
    pose.pose.position.y = path_points[i].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = yawToQuaternion(yawAt(path_points, i));
    path.poses.push_back(pose);
  }
  return path;
}

double LineDrawTool::yawAt(const std::vector<Point2D> & path_points, const std::size_t index) const
{
  if (path_points.size() < 2) {
    return 0.0;
  }
  const std::size_t prev = index == 0 ? 0 : index - 1;
  const std::size_t next = std::min(path_points.size() - 1, index + 1);
  return std::atan2(path_points[next].y - path_points[prev].y, path_points[next].x - path_points[prev].x);
}

double LineDrawTool::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void LineDrawTool::publishMarkers()
{
  if (!marker_pub_) {
    return;
  }
  clearMarkers();

  visualization_msgs::msg::MarkerArray markers;
  const auto stamp = raw_node_->now();

  visualization_msgs::msg::Marker clicked;
  clicked.header.stamp = stamp;
  clicked.header.frame_id = "map";
  clicked.ns = "clicked_points";
  clicked.id = 0;
  clicked.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  clicked.action = visualization_msgs::msg::Marker::ADD;
  clicked.scale.x = 0.45;
  clicked.scale.y = 0.45;
  clicked.scale.z = 0.45;
  clicked.color.r = 1.0;
  clicked.color.g = 0.15;
  clicked.color.b = 0.05;
  clicked.color.a = 1.0;
  for (const auto & p : clicked_points_) {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = 0.05;
    clicked.points.push_back(point);
  }
  markers.markers.push_back(clicked);

  visualization_msgs::msg::Marker line;
  line.header.stamp = stamp;
  line.header.frame_id = "map";
  line.ns = "clicked_line";
  line.id = 1;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = line_width_property_->getFloat();
  line.color.r = 0.0;
  line.color.g = 0.75;
  line.color.b = 1.0;
  line.color.a = 1.0;
  for (const auto & p : clicked_points_) {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = 0.03;
    line.points.push_back(point);
  }
  markers.markers.push_back(line);

  last_marker_count_ = static_cast<int>(markers.markers.size());
  marker_pub_->publish(markers);
}

void LineDrawTool::clearMarkers()
{
  if (!marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  for (int id = 0; id < std::max(2, last_marker_count_); ++id) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = raw_node_->now();
    marker.header.frame_id = "map";
    marker.ns = id == 0 ? "clicked_points" : "clicked_line";
    marker.id = id == 0 ? 0 : 1;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(marker);
  }
  marker_pub_->publish(markers);
}

void LineDrawTool::recreateMarkerPublisher()
{
  if (!raw_node_) {
    return;
  }
  marker_pub_ = raw_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_property_->getStdString(), rclcpp::QoS(1).transient_local());
}

void LineDrawTool::recreateClickedPointsPublisher()
{
  if (!raw_node_) {
    return;
  }
  clicked_points_pub_ = raw_node_->create_publisher<nav_msgs::msg::Path>(
    clicked_points_topic_property_->getStdString(), rclcpp::QoS(1).reliable().transient_local());
}

void LineDrawTool::handleCommand(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (msg->data == kCmdDraw) {
    setDrawingEnabled(true);
    return;
  }
  if (msg->data == kCmdPublish) {
    publishAll();
    return;
  }
  if (msg->data == kCmdClear) {
    clearAll();
  }
}

void LineDrawTool::setDrawingEnabled(const bool enabled)
{
  drawing_enabled_ = enabled;
  updateControlPane();
  publishStatus(enabled ? "已进入打点模式：在地图中左键连续打点。" : "已退出打点模式。");
}

void LineDrawTool::publishStatus(const std::string & text)
{
  setStatus(text.c_str());
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
  if (status_label_) {
    status_label_->setText(QString::fromStdString(text));
  }
}

void LineDrawTool::createControlPane()
{
  if (control_dock_) {
    updateControlPane();
    return;
  }
  if (!context_) {
    return;
  }
  auto * window_manager = context_->getWindowManager();
  if (!window_manager) {
    return;
  }

  auto * pane = new QWidget();
  auto * layout = new QVBoxLayout(pane);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto * title = new QLabel("路径连续打点", pane);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);

  point_count_label_ = new QLabel("点数: 0", pane);
  mode_label_ = new QLabel("模式: 未打点", pane);
  status_label_ = new QLabel("点击“打点”后，在地图中左键连续打点。", pane);
  status_label_->setWordWrap(true);

  auto * button_row = new QHBoxLayout();
  draw_button_ = new QPushButton("打点", pane);
  publish_button_ = new QPushButton("发布", pane);
  clear_button_ = new QPushButton("清除", pane);
  button_row->addWidget(draw_button_);
  button_row->addWidget(publish_button_);
  button_row->addWidget(clear_button_);

  QObject::connect(draw_button_, &QPushButton::clicked, [this]() {
    selectThisTool();
    setDrawingEnabled(true);
  });
  QObject::connect(publish_button_, &QPushButton::clicked, [this]() {
    publishAll();
  });
  QObject::connect(clear_button_, &QPushButton::clicked, [this]() {
    clearAll();
  });

  layout->addWidget(title);
  layout->addWidget(point_count_label_);
  layout->addWidget(mode_label_);
  layout->addLayout(button_row);
  layout->addWidget(status_label_);
  layout->addStretch(1);

  control_dock_ = window_manager->addPane(
    "LineDraw 控制",
    pane,
    Qt::LeftDockWidgetArea,
    false);
  control_dock_->show();
  updateControlPane();
}

void LineDrawTool::updateControlPane()
{
  if (point_count_label_) {
    point_count_label_->setText(
      QString("点数: %1").arg(static_cast<int>(clicked_points_.size())));
  }
  if (mode_label_) {
    mode_label_->setText(QString("模式: %1").arg(drawing_enabled_ ? "打点中" : "未打点"));
  }
}

void LineDrawTool::clearAll()
{
  clicked_points_.clear();
  if (request_pub_) request_pub_->publish(buildPathMsg({}));
  drawing_enabled_ = false;
  clearMarkers();
  publishMarkers();
  publishClickedPoints();
  updateControlPane();
  publishStatus("已清除路径并请求停车。");
}

void LineDrawTool::selectThisTool()
{
  if (!context_) {
    return;
  }
  auto * tool_manager = context_->getToolManager();
  if (!tool_manager) {
    return;
  }
  tool_manager->setCurrentTool(this);
}

}  // namespace test_c

PLUGINLIB_EXPORT_CLASS(test_c::LineDrawTool, rviz_common::Tool)
