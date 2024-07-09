/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsAtracsysFusionTrack_h
#define _mtsAtracsysFusionTrack_h

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmPositionCartesianArrayGet.h>
#include <cisstParameterTypes/prmImageFrame.h>
#include <cisstParameterTypes/prmCameraInfo.h>

#include <json/json.h> // in order to read config file

#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackExport.h>  // always include last

// forward declarations for internal data
class mtsAtracsysFusionTrackInternals;
class mtsAtracsysFusionTrackTool;

/*!
  \todo Create InitLibrary method and call it in Configure or AddTool is not already done
  \todo Add flag to check if ftkInit has been called and then check in AddTool and Startup, report error if not
  \todo Add method AddTool to add tool from geometry as std::vector<vct3> + geometry Id
  \todo Use method AddTool in AddToolIni
  \todo Can this thing beep on command?
*/
class CISST_EXPORT mtsAtracsysFusionTrack: public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    inline mtsAtracsysFusionTrack(const std::string & componentName):
        mtsTaskContinuous(componentName, 100),
        m_configuration_state_table(100, "configuration") {
        Init();
    }

    inline mtsAtracsysFusionTrack(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg),
        m_configuration_state_table(100, "configuration") {
        Init();
    }

    ~mtsAtracsysFusionTrack(void) {};

    /*! Configure the device.  If the method is called without a file
      name (or an empty string), this method initializes the hardware.
      Users will have to call the AddTool later to add tools.  If a
      file name is provided, the methods assumes it corresponds to a
      JSON file containing an array of "tools", each of them being
      defined by a "name" and an "ini-file" or "json-file".  The path
      for the ini/json geometry file can be either absolute or
      relative to the application's working directory. */
    void Configure(const std::string & filename = "") override;

    void Startup(void) override;

    void Run(void) override;

    void Cleanup(void) override;

    /*! Create a tool using the tool geometry file.  By default,
      Atracsys provides .ini files but we also support .json. */
    bool AddTool(const std::string & toolName,
                 const std::string & fileName,
                 const bool isJson,
                 const std::string & referenceName);

    /*! For backward compatibility */
    inline bool AddToolIni(const std::string & toolName,
                           const std::string & fileName) {
        return AddTool(toolName, fileName, false, "");
    }

    inline size_t GetNumberOfTools(void) const {
        return m_tools.size();
    }

    /*! Set the maximum number of stray markers to look for. */
    inline void SetStrayMarkerMax(const size_t max) {
        m_stray_markers_max = max;
    }

    std::string GetToolName(const size_t index) const;

protected:

    /*! Code called by all constructors. */
    void Init(void);

    void ProcessIRTrackingFrame();
    void ProcessRGBStereoFrame();

    /*! Search path for configuration files */
    cmnPath m_path;
    
    /*! State table for configuration */
    mtsStateTable m_configuration_state_table;

    /*! Controller interface */
    mtsInterfaceProvided * m_controller_interface = nullptr;

    mtsInterfaceProvided * m_stereo_raw_interface = nullptr;
    
    mtsAtracsysFusionTrackInternals * m_internals = nullptr;
    typedef std::map<std::string, mtsAtracsysFusionTrackTool *> ToolsType;
    ToolsType m_tools;

    /*! Number of stray markers to track */
    size_t m_stray_markers_max = 0;

    prmPositionCartesianArrayGet m_measured_cp_array;

    prmCameraInfo m_left_camera_info, m_right_camera_info;
    vct3 m_camera_rotation, m_camera_translation;
    prmImageFrame m_left_image_raw, m_right_image_raw;
    int m_num_enabled_dot_projectors = 0;

    /*! CRTK related methods */
    mtsFunctionVoid m_crtk_interfaces_provided_updated;
    std::vector<mtsDescriptionInterfaceFullName> m_crtk_interfaces_provided;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsAtracsysFusionTrack);

#endif  // _mtsAtracsysFusionTrack_h
