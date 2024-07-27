#ifndef _POLYGONSELECTTOOL_HPP_
#define _POLYGONSELECTTOOL_HPP_

#include <memory>
#include <string>
#include <utility>

#include <OgreVector3.h>

#include <QCursor>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/line.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace scc_rviz_plugin
{
class PolygonSelectTool : public rviz_common::Tool
{
public:
    PolygonSelectTool();
    ~PolygonSelectTool() override;
    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

    Ogre::ColourValue pointColor;
    Ogre::ColourValue lineColor;
protected:
    virtual void onPointSet(double x, double y) = 0;
    virtual int onPolygonSet(std::vector<Ogre::Vector3> & p) = 0;

    void logPoint(std::string designation, geometry_msgs::msg::Point position, std::string frame);

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
    
    Ogre::Vector3 current_point_position_;
    std::vector<Ogre::Vector3> point_positions_;

    std::shared_ptr<rviz_rendering::Shape> current_point_marker_;
    std::vector<std::shared_ptr<rviz_rendering::Shape>> point_markers_;
    std::vector<std::shared_ptr<rviz_rendering::Line>> line_markers_;
private:
    int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
    int processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
    int processMouseLeftButtonReleased();
    int processMouseRightButtonPressed();

    void showMarker(bool show);
};
} // namespace scc_rviz_plugin

#endif // _POLYGONSELECTTOOL_HPP_