#ifndef _POINT_SELECT_TOOL_HPP_
#define _POINT_SELECT_TOOL_HPP_

#include <memory>
#include <string>
#include <utility>

#include <OgreVector3.h>

#include <QCursor>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace scc_rviz_plugin
{

class PointSelecet : public rviz_common::Tool
{
public:
    PointSelecet();
    ~PointSelecet() override;
    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
protected:
    virtual int onPointSet(double x, double y) = 0;
    void logPoint(std::string designation, geometry_msgs::msg::Point position, std::string frame);
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
    std::shared_ptr<rviz_rendering::Shape> point_marker_;
    Ogre::Vector3 point_position_;
private:
    int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
    int processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
    int processMouseLeftButtonReleased();
    void showMarker(bool show);
};
} // namespace scc_rviz_plugin

#endif // _POSE_SELECT_TOOL_HPP_