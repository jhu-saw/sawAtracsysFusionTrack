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

// cisst
#include <cisst_ros_bridge/mtsROSBridge.h>

CMN_IMPLEMENT_SERVICES(atracsys_bridge);

void atracsys_bridge::bridge(const std::string & _component_name,
                                          const std::string & _interface_name,
                                          const double _publish_period_in_seconds,
                                          const double _tf_period_in_seconds)
{
    // clean ROS namespace
    std::string _clean_namespace = _component_name;
    cisst_ral::clean_namespace(_clean_namespace);

    // create factory to bridge tool as they get created
    add_factory_source(_component_name,
                             _interface_name,
                             _publish_period_in_seconds,
                             _tf_period_in_seconds);

    // controller specific topics, some might be CRTK compliant
    bridge_interface_provided(_component_name,
                                    _interface_name,
                                    _clean_namespace,
                                    _publish_period_in_seconds,
                                    _tf_period_in_seconds);
}

void atracsys_bridge::bridge_interface_provided(const std::string & _component_name,
                                          const std::string & _interface_name,
                                          const double _publish_period_in_seconds,
                                          const double _tf_period_in_seconds) {
    // bridge CRTK topics
    mts_ros_crtk_bridge::bridge_interface_provided(_component_name, _interface_name,
                                                   _publish_period_in_seconds, _tf_period_in_seconds);
    // and also the registration_error command (if present)
    bridge_tool_error(_component_name, _interface_name);
}

void atracsys_bridge::bridge_tool_error(const std::string & _component_name,
                                                     const std::string & _interface_name)
{
    mtsManagerLocal * _component_manager = mtsComponentManager::GetInstance();
    mtsComponent * _component = _component_manager->GetComponent(_component_name);
    if (!_component) {
        CMN_LOG_CLASS_RUN_ERROR << "bridge_tool_error: unable to find component \""
                                 << _component_name << "\"" << std::endl;
        return;
    }
    // then try to find the interface
    mtsInterfaceProvided * _interface_provided = _component->GetInterfaceProvided(_interface_name);

    // check if interface provides tool registration error
    const auto& read_commands = _interface_provided->GetNamesOfCommandsRead();
    if (std::find(read_commands.begin(), read_commands.end(), "registration_error") == read_commands.end()) {
        return;
    }

    std::string ros_topic = _component_name + "/" + _interface_name + "/registration_error";

    m_subscribers_bridge->AddPublisherFromCommandRead<double, CISST_RAL_MSG(std_msgs, Float64)>
        (_interface_name, "registration_error", ros_topic);
}
