/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsAtracsysFusionTrack_h
#define _mtsAtracsysFusionTrack_h

#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackExport.h>  // always include last

// forward declarations for internal data
class mtsAtracsysFusionTrackInternals;
class mtsAtracsysFusionTrackTool;

class CISST_EXPORT mtsAtracsysFusionTrack: public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsAtracsysFusionTrack(const std::string & componentName):
        mtsTaskContinuous(componentName, 100) { Construct(); }

    mtsAtracsysFusionTrack(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg) { Construct(); }

    ~mtsAtracsysFusionTrack(void) {};

    void Construct(void);

    void Configure(const std::string & filename = "");

    void Startup(void) {};

    void Run(void);

    void Cleanup(void);

    bool AddToolIni(const std::string & toolName, const std::string & fileName);

    size_t GetNumberOfTools(void) const {
        return Tools.size();
    }

    std::string GetToolName(const size_t index) const;

protected:
    mtsAtracsysFusionTrackInternals * Internals;
    typedef cmnNamedMap<mtsAtracsysFusionTrackTool> ToolsType;
    ToolsType Tools;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsAtracsysFusionTrack);

#endif  // _mtsAtracsysFusionTrack_h
