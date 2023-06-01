
#ifndef SET_POI_H
#define SET_POI_H


#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include <rclcpp/rclcpp.hpp>
# include <rviz_common/tool.hpp>
#endif
#include <navigation_assistant/srv/nav_assistant_poi.hpp>
#include <navigation_assistant/srv/nav_assistant_point.hpp>


namespace POI
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
      rclcpp::Client<navigation_assistant::srv::NavAssistantPoint>::SharedPtr nav_assist_srv_client;
      bool shift_down;
    };

}

#endif

