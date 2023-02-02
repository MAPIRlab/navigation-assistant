
#ifndef SET_POI_H
#define SET_POI_H


#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include <ros/ros.h>
# include <rviz/tool.h>
#endif

namespace rviz
{
    class VectorProperty;
    class VisualizationManager;
    class ViewportMouseEvent;
}

namespace POI
{
    // declare our new subclass of rviz::Tool
    class SetPointOfInterestTool : public rviz::Tool
    {
    Q_OBJECT
    public:
      SetPointOfInterestTool();
      virtual ~SetPointOfInterestTool() {}

      virtual void onInitialize();

      virtual void activate();

      virtual void deactivate();

      virtual int processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel);
      virtual int processMouseEvent(rviz::ViewportMouseEvent& event) override;

    private:
      ros::NodeHandle nh_;
      ros::ServiceClient nav_assist_srv_client;
      bool shift_down;
    };

}

#endif

