/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-11-28

  (C) Copyright 2017-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _atracsys_bridge_h
#define _atracsys_bridge_h

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

class atracsys_bridge: public mts_ros_crtk_bridge_provided
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    atracsys_bridge(const std::string& component_name,
                    cisst_ral::node_ptr_t node_handle,
                    double ros_period = 5.0 * cmn_ms,
                    double tf_period = 5.0 * cmn_ms);

    void bridge_controller(const std::string& _component_name,
                const std::string& _interface_name);

    void bridge_stereo(const std::string & _component_name,
                const std::string & _interface_name,
                const std::string & _ros_namespace);

protected:
    mtsROSBridge* m_pub_bridge;

    const double m_ros_period;
    const double m_tf_period;

    // source factory for tools
    mtsFunctionRead m_get_factory_sources;
    std::set<mtsDescriptionInterfaceFullName> m_bridged_sources;
    void sources_update_handler();

    void bridge_tool(const std::string & _component_name,
                     const std::string & _interface_name,
                     const std::string & _ros_namespace);
};

CMN_DECLARE_SERVICES_INSTANTIATION(atracsys_bridge);

#endif // _atracsys_bridge_h
