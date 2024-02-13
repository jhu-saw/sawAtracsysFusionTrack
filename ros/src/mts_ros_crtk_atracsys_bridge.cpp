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

// system include
#include <iostream>

// cisst
#include "mts_ros_crtk_atracsys_bridge.h"
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <std_msgs/Float64.h>

CMN_IMPLEMENT_SERVICES(mts_ros_crtk_atracsys_bridge);

void mts_ros_crtk_atracsys_bridge::bridge(const std::string & _component_name,
                                          const std::string & _interface_name,
                                          const double _publish_period_in_seconds,
                                          const double _tf_period_in_seconds)
{
    // clean ROS namespace
    std::string _clean_namespace = _component_name;
    cisst_ral::clean_namespace(_clean_namespace);

    // create factory to bridge tool as they get created
    this->add_factory_source(_component_name,
                             _interface_name,
                             _publish_period_in_seconds,
                             _tf_period_in_seconds);

    // controller specific topics, some might be CRTK compliant
    this->bridge_interface_provided(_component_name,
                                    _interface_name,
                                    _clean_namespace,
                                    _publish_period_in_seconds,
                                    _tf_period_in_seconds);

    // non CRTK topics
    // add trailing / for clean namespace
    if (!_clean_namespace.empty()) {
        _clean_namespace.append("/");
    }
    // required interfaces specific to this component to bridge
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    // connections
    m_connections.Add(m_subscribers_bridge->GetName(), _required_interface_name,
                      _component_name, _interface_name);
    m_connections.Add(m_events_bridge->GetName(), _required_interface_name,
                      _component_name, _interface_name);
}

void mts_ros_crtk_atracsys_bridge::bridge_tool_error(const std::string & _component_name,
                                                     const std::string & _interface_name)
{
    std::string _clean_namespace = _component_name;
    cisst_ral::clean_namespace(_clean_namespace);

    // add trailing / for clean namespace
    if (!_clean_namespace.empty()) {
        _clean_namespace.append("/");
    }
    // required interfaces specific to this component to bridge
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    // non CRTK topics
    m_subscribers_bridge->AddPublisherFromCommandRead<double, std_msgs::Float64>
        (_required_interface_name, "registration_error", _component_name + "/" + _interface_name + "/registration_error");

    m_connections.Add(m_subscribers_bridge->GetName(), _required_interface_name,
                      _component_name, _interface_name);
}
