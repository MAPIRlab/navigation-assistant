#include "set_point_of_interest_tool.h"
#include <nav_assistant_msgs/srv/nav_assistant_point.hpp>


#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <QKeyEvent>


//if you do <OGRE/OgreSceneNode.h> you end up finding the wrong version (system install rather than rviz_ogre_vendor) 
//because the include path exported by the ament rviz package is ..../install/OGRE rather than just ..../install  
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h> 

namespace rviz_nav_assistant
{

    SetPointOfInterestTool::SetPointOfInterestTool() : Node("NavAssistantPOITool")
    {
        // Set shorcut key
        shortcut_key_ = 'p';
        nav_assist_srv_client = create_client<nav_assistant_msgs::srv::NavAssistantPoint>("/navigation_assistant/add_point_of_interest");
        shift_down = false;
    }

    void SetPointOfInterestTool::onInitialize()
    {
    }

    void SetPointOfInterestTool::activate()
    {
    }

    void SetPointOfInterestTool::deactivate()
    {
    }

    // Handling key events
    // ^^^^^^^^^^^^^^^^^^^^^
    int SetPointOfInterestTool::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel)
    {
        if(event->type() == QKeyEvent::KeyPress)
        {
            if( event->key() == Qt::Key_Shift )
                shift_down = true;
            else if ( event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)
            {
                //Save graph
                nav_assistant_msgs::srv::NavAssistantPoint::Request::SharedPtr request;
                request->action = "save";
                auto future = nav_assist_srv_client->async_send_request(request);
            }
        }
        else //if(event->type() == QKeyEvent::KeyRelease)
        {
            if( event->key() == Qt::Key_Shift )
                shift_down = false;
        }
        return 1;
    }

    // Handling mouse events
    // ^^^^^^^^^^^^^^^^^^^^^
    int SetPointOfInterestTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
    {
        // processMouseEvent() is sort of the main function of a Tool, because
        // mouse interactions are the point of Tools.

        if (event.leftDown())   //Add/remove a new Interesting Point (space)
        {
            //1. Get (x,y,yaw) in the plane z=0
            Ogre::Vector3 intersection;
            Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
            if( context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, intersection))
            {
                nav_assistant_msgs::srv::NavAssistantPoint::Request::SharedPtr request;
                if (event.shift())
                    request->action = "delete";
                else
                    request->action = "add";
                request->type = "space";
                request->pose.pose.position.x = intersection.x;
                request->pose.pose.position.y = intersection.y;
                request->pose.pose.position.z = 0.0;
                request->pose.pose.orientation.w = 1.0;
                //ROS_INFO("[POI]: Requesting the creation of new [%s] at (%.3f, %.3f)  ", srv_call.request.type.c_str() , intersection.x, intersection.y );

                nav_assist_srv_client->async_send_request(request);
            }
        }        
        else if (event.rightDown()) //Add a new Interesting Point (CNP)
        {
            //1. Get (x,y,yaw) in the plane z=0
            Ogre::Vector3 intersection;
            Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
            if( context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, intersection) )
            {
                nav_assistant_msgs::srv::NavAssistantPoint::Request::SharedPtr request;
                if (event.shift())
                    request->action = "delete";
                else
                    request->action = "add";
                request->type = "CNP";
                request->pose.pose.position.x = intersection.x;
                request->pose.pose.position.y = intersection.y;
                request->pose.pose.position.z = 0.0;
                request->pose.pose.orientation.w = 1.0;
                //ROS_INFO("[POI]: Requesting the creation of new [%s] at (%.3f, %.3f)  ", srv_call.request.type.c_str() , intersection.x, intersection.y );
                nav_assist_srv_client->async_send_request(request);
            }
        }
        else if (event.middleDown())
        {
        }
        return 1;
    }

} //end namespace


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_nav_assistant::SetPointOfInterestTool, rviz_common::Tool )
