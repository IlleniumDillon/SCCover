#ifndef NSCC_COVER_TASK_HPP
#define NSCC_COVER_TASK_HPP

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

// #include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "_PointSelectTool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "scc_message/msg/cover_task.hpp"

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace scc_rviz_plugin
{
class NSCCCoverTask : public PointSelecet
{
    Q_OBJECT
public:
    NSCCCoverTask();
    ~NSCCCoverTask() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
protected:
    int onPointSet(double x, double y) override;
private Q_SLOTS:
    void updateTopic();
private:
    rclcpp::Publisher<scc_message::msg::CoverTask>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty * pub_topic_property_;
    rviz_common::properties::QosProfileProperty * pub_qos_profile_property_;

    rclcpp::QoS qos_profile_;

    int counter_;

    scc_message::msg::CoverTask msg;
};
}  // namespace scc_rviz_plugin

#endif // NCC_COVER_TASK_HPP