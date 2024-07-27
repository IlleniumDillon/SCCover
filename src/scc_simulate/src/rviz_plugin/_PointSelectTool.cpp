#include "_PointSelectTool.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace scc_rviz_plugin
{
PointSelecet::PointSelecet()
    : rviz_common::Tool(), point_marker_(nullptr), point_position_(Ogre::Vector3::ZERO)
{
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

PointSelecet::~PointSelecet() = default;

void PointSelecet::onInitialize()
{
    point_marker_ = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_);
    point_marker_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    point_marker_->setScale(Ogre::Vector3(0.1, 0.1, 0.1));
    point_marker_->setOffset(Ogre::Vector3(0, 0, 0.05));
    point_marker_->getRootNode()->setVisible(false);
}

void PointSelecet::activate()
{ 
    setStatus("Click on the map to select a point.");
}
void PointSelecet::deactivate()
{
    point_marker_->getRootNode()->setVisible(false);
    point_position_ = Ogre::Vector3::ZERO;
}
int PointSelecet::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
    auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

    if (event.leftDown()) 
    {
        //RVIZ_COMMON_LOG_INFO_STREAM("Left mouse button pressed");
        return processMouseLeftButtonPressed(point_projection_on_xy_plane);
    } 
    else if (event.type == QEvent::MouseMove && event.left()) 
    {
        //RVIZ_COMMON_LOG_INFO_STREAM("Mouse moved");
        return processMouseMoved(point_projection_on_xy_plane);
    } 
    else if (event.leftUp()) 
    {
        //RVIZ_COMMON_LOG_INFO_STREAM("Left mouse button released");
        return processMouseLeftButtonReleased();
    }

    return 0;
}

void PointSelecet::logPoint(std::string designation, geometry_msgs::msg::Point position, std::string frame)
{
    RVIZ_COMMON_LOG_INFO_STREAM(designation << " point: (" << position.x << ", " << position.y << ", " << position.z << ") in frame " << frame);    
}
int PointSelecet::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
    int flags = 0;
    if (xy_plane_intersection.first) 
    {
        point_position_ = xy_plane_intersection.second;
        point_marker_->setPosition(point_position_);

        point_marker_->getRootNode()->setVisible(true);

        flags |= Render;
    }
    return flags;
}

int PointSelecet::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
{
    return Render;
}
int PointSelecet::processMouseLeftButtonReleased()
{
    int flags = 0;

    flags |= onPointSet(point_position_.x, point_position_.y);

    return flags;
}

void PointSelecet::showMarker(bool show)
{
    point_marker_->getRootNode()->setVisible(show);
}

} // namespace scc_rviz_plugin