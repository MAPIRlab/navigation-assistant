
#ifndef SET_POI_H
#define SET_POI_H


#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include <rclcpp/rclcpp.hpp>
# include <rviz_common/tool.hpp>
#endif
#include <nav_assistant_msgs/srv/nav_assistant_poi.hpp>
#include <nav_assistant_msgs/srv/nav_assistant_point.hpp>


namespace rviz_nav_assistant
{
    // declare our new subclass of rviz::Tool
    class SetPointOfInterestTool : public rviz_common::Tool, public rclcpp::Node
    {
    Q_OBJECT
    public:
      SetPointOfInterestTool();
      virtual ~SetPointOfInterestTool() {}

      virtual void onInitialize();

      virtual void activate();

      virtual void deactivate();

      virtual int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel);
      virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

    private:
      rclcpp::Client<nav_assistant_msgs::srv::NavAssistantPoint>::SharedPtr nav_assist_srv_client;
      bool shift_down;
    };

}

#endif

