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

#include <ftkInterface.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrack.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAtracsysFusionTrack, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

class mtsAtracsysFusionTrackInternals
{
public:
    mtsAtracsysFusionTrackInternals(void) :
        Library(0),
        Device(0)
    {};
    ftkLibrary Library;
    uint64 Device;
};


void mtsAtracsysFusionTrackDeviceEnum(uint64 sn, void * user, ftkDeviceType type)
{
    uint64 * lastDevice =reinterpret_cast<uint64 *>(user);
    if (lastDevice) {
        *lastDevice = sn;
    }
}


void mtsAtracsysFusionTrack::Construct(void)
{
    Internals = new mtsAtracsysFusionTrackInternals();

    mtsInterfaceProvided * provided = AddInterfaceProvided("Controller");
    if (provided) {
//        provided->AddCommandReadState(StateTable, IsTracking, "IsTracking");
    }
}


void mtsAtracsysFusionTrack::Configure(const std::string & filename)
{
    Internals->Library = ftkInit();
    if (!Internals->Library) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to initialize (" << this->GetName() << ")" << std::endl;
        return;
    }

   
    // Scan for devices
    ftkError error = ftkEnumerateDevices(Internals->Library,
                                         mtsAtracsysFusionTrackDeviceEnum,
                                         &(Internals->Device));
    if (error != FTK_OK) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to enumerate devices (" << this->GetName() << ")" << std::endl;
        ftkClose(Internals->Library);
    }
    if (Internals->Device == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: no device connected (" << this->GetName() << ")" << std::endl;
        ftkClose(Internals->Library);
        return;
    }
    std::cerr << "configure" << std::endl;
}


void mtsAtracsysFusionTrack::Run(void)
{
    // std::cerr << "running" << std::endl;
    ProcessQueuedCommands();
}


void mtsAtracsysFusionTrack::Cleanup(void)
{
}


#if 0
mtsAtracsysFusionTrack::Tool * mtsAtracsysFusionTrack::AddTool(const std::string & name, const char * serialNumber)
{
    Tool * tool = CheckTool(serialNumber);

    if (tool) {
        CMN_LOG_CLASS_INIT_WARNING << "AddTool: " << tool->Name << " already exists, renaming it to " << name << " instead" << std::endl;
        tool->Name = name;
    } else {
        tool = new Tool();
        tool->Name = name;
        strncpy(tool->SerialNumber, serialNumber, 8);

        if (!Tools.AddItem(tool->Name, tool, CMN_LOG_LEVEL_INIT_ERROR)) {
            CMN_LOG_CLASS_INIT_ERROR << "AddTool: no tool created, duplicate name exists: " << name << std::endl;
            delete tool;
            return 0;
        }
        CMN_LOG_CLASS_INIT_VERBOSE << "AddTool: created tool \"" << name << "\" with serial number: " << serialNumber << std::endl;

        // create an interface for tool
        tool->Interface = AddInterfaceProvided(name);
        if (tool->Interface) {
            StateTable.AddData(tool->TooltipPosition, name + "Position");
            tool->Interface->AddCommandReadState(StateTable, tool->TooltipPosition, "GetPositionCartesian");
            StateTable.AddData(tool->MarkerPosition, name + "Marker");
            tool->Interface->AddCommandReadState(StateTable, tool->MarkerPosition, "GetMarkerCartesian");
        }
    }
    return tool;
}
#endif

std::string mtsAtracsysFusionTrack::GetToolName(const size_t index) const
{
    ToolsType::const_iterator toolIterator = Tools.begin();
    if (index >= Tools.size()) {
        CMN_LOG_CLASS_RUN_ERROR << "GetToolName: requested index is out of range" << std::endl;
        return "";
    }
    for (unsigned int i = 0; i < index; i++) {
        toolIterator++;
    }
    return toolIterator->first;
}
