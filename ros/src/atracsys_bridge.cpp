/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-11-28

  (C) Copyright 2017-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "atracsys_bridge.h"

#include <algorithm>
#include <iostream>

#include <cisst_ros_bridge/cisst_ral.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisstMultiTask/mtsManagerComponentServices.h>

CMN_IMPLEMENT_SERVICES(atracsys_bridge);

atracsys_bridge::atracsys_bridge(const std::string& _component_name,
                                 cisst_ral::node_ptr_t _node_handle,
                                 double ros_period,
                                 double tf_period):
    mts_ros_crtk_bridge_provided(_component_name, _node_handle, ros_period),
    m_ros_period(ros_period),
    m_tf_period(tf_period)
{
    mtsManagerLocal* componentManager = mtsManagerLocal::GetInstance();

    std::string m_bridge_name = GetName() + "_pub";
    cisst_ral::clean_namespace(m_bridge_name);

    m_pub_bridge = new mtsROSBridge(m_bridge_name, ros_period, _node_handle);
    m_pub_bridge->AddIntervalStatisticsInterface();
    componentManager->AddComponent(m_pub_bridge);

    // add pub bridge stats to the stats bridge from the base class
    stats_bridge().AddIntervalStatisticsPublisher("publishers", m_pub_bridge->GetName());
}

void atracsys_bridge::bridge_controller(const std::string & _component_name,
                                        const std::string & _interface_name)
{
    std::string required_interface_name = GetName() + "_" + _component_name + "_using_" + _interface_name + "_factory";
    mtsInterfaceRequired* required_interface = AddInterfaceRequired(required_interface_name);

    if (!required_interface) {
        CMN_LOG_CLASS_INIT_ERROR << "bridge_controller: couldn't add required interface for factory source"  << std::endl;
    }

    required_interface->AddFunction("crtk_interfaces_provided", m_get_factory_sources);
    required_interface->AddEventHandlerVoid(&atracsys_bridge::sources_update_handler, this,
                                  "crtk_interfaces_provided_updated");

    mtsManagerLocal* componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(GetName(), required_interface_name, _component_name, _interface_name);

    // clean ROS namespace
    std::string clean_namespace = _component_name;
    cisst_ral::clean_namespace(clean_namespace);

    // bridge all CRTK-compatible topics from the controller
    bridge_interface_provided(_component_name, _interface_name, clean_namespace,
                              m_ros_period, m_tf_period);
}

void atracsys_bridge::bridge_stereo(const std::string & _component_name,
                                    const std::string & _interface_name,
                                    const std::string & _ros_namespace)
{
    std::string clean_namespace = _ros_namespace;
    cisst_ral::clean_namespace(clean_namespace);

    // bridge all CRTK-compatible topics from the controller
    bridge_interface_provided(_component_name, _interface_name, clean_namespace,
                              m_ros_period, m_tf_period);

    m_pub_bridge->AddPublisherFromCommandRead<prmImageFrame, CISST_RAL_MSG(sensor_msgs, Image)>
        (_interface_name, "left/image_rect_color", clean_namespace + "/left/image_rect_color");

    m_pub_bridge->AddPublisherFromCommandRead<prmImageFrame, CISST_RAL_MSG(sensor_msgs, Image)>
        (_interface_name, "right/image_rect_color", clean_namespace + "/right/image_rect_color");

    m_pub_bridge->AddPublisherFromCommandRead<prmCameraInfo, CISST_RAL_MSG(sensor_msgs, CameraInfo)>
        (_interface_name, "left/camera_info", clean_namespace + "/left/camera_info");

    m_pub_bridge->AddPublisherFromCommandRead<prmCameraInfo, CISST_RAL_MSG(sensor_msgs, CameraInfo)>
        (_interface_name, "right/camera_info", clean_namespace + "/right/camera_info");

    m_pub_bridge->AddPublisherFromCommandRead<prmDepthMap, CISST_RAL_MSG(sensor_msgs, PointCloud2)>
        (_interface_name, "depth", clean_namespace + "/depth");

    mtsManagerLocal* componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(m_pub_bridge->GetName(), _interface_name, _component_name, _interface_name);
}

void atracsys_bridge::bridge_tool(const std::string & _component_name,
                                  const std::string & _interface_name,
                                  const std::string & _ros_namespace)
{
    std::string clean_namespace = _ros_namespace;
    cisst_ral::clean_namespace(clean_namespace);
    std::string ros_topic = clean_namespace + "/registration_error";

    m_pub_bridge->AddPublisherFromCommandRead<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_interface_name, "registration_error", ros_topic);

    mtsManagerLocal* componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(m_pub_bridge->GetName(), _interface_name, _component_name, _interface_name);
}

void atracsys_bridge::sources_update_handler()
{
    std::vector<mtsDescriptionInterfaceFullName> sources;
    m_get_factory_sources(sources);
    for (const auto & source : sources) {
        // skip any sources bridged previously
        if (m_bridged_sources.find(source) != m_bridged_sources.end()) {
            continue;
        }

        std::string ros_namespace = source.ComponentName + "/" + source.InterfaceName;
        cisst_ral::clean_namespace(ros_namespace);
        bridge_interface_provided(source.ComponentName,
                                  source.InterfaceName,
                                  ros_namespace,
                                  m_ros_period, m_tf_period);
        bridge_tool(source.ComponentName, source.InterfaceName, ros_namespace);
        Connect();
        m_bridged_sources.insert(source);
    }
}
