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

#include <geometryHelper.hpp> // I would like to get rid of this and use only ftk functions + remove atracsys_DIR/bin from include directories

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrack.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAtracsysFusionTrack, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

class mtsAtracsysFusionTrackInternals
{
public:
    mtsAtracsysFusionTrackInternals(const size_t numberOfMarkers = 32):
        NumberOfMarkers(numberOfMarkers),
        Library(0),
        Device(0)
    {
        memset(&Frame, 0, sizeof(ftkFrameQuery));
        Markers = new ftkMarker[NumberOfMarkers];
        Frame.markers = Markers;
        Frame.markersVersionSize.ReservedSize = sizeof(ftkMarker) * NumberOfMarkers;
    };

    size_t NumberOfMarkers;
    ftkLibrary Library;
    uint64 Device;
    ftkFrameQuery Frame;
    ftkMarker * Markers;
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

    // to be moved to AddTool
    ftkGeometry geom;

    switch (loadGeometry(Internals->Library, Internals->Device, "geometry003.ini", geom))
    {
    case 1:
        std::cerr << "Loaded from installation directory." << std::endl;
    case 0:
        error = ftkSetGeometry(Internals->Library, Internals->Device, &geom);
        if (error != FTK_OK) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to set geometry (" << this->GetName() << ")" << std::endl;
        }
        break;
    default:
        CMN_LOG_CLASS_INIT_ERROR << "Error, cannot load geometry file." << std::endl;
    }

    std::cerr << "configure" << std::endl;
}


void mtsAtracsysFusionTrack::Run(void)
{
    ProcessQueuedCommands();
    if (ftkGetLastFrame(Internals->Library,
                        Internals->Device,
                        &(Internals->Frame),
                        100 /* block up to 100 ms if next frame is not available*/) != FTK_OK) {
        CMN_LOG_CLASS_RUN_DEBUG << "Run: timeout on ftkGetLastFrame" << std::endl;
        return;
    }

    switch (Internals->Frame.markersStat) {
    case QS_WAR_SKIPPED:
        CMN_LOG_CLASS_RUN_ERROR << "Run: marker fields in the frame are not set correctly" << std::endl;
    case QS_ERR_INVALID_RESERVED_SIZE:
        CMN_LOG_CLASS_RUN_ERROR << "Run: frame.markersVersionSize is invalid" << std::endl;
    default:
        CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
    case QS_OK:
        break;
    }

    size_t count = Internals->Frame.markersCount;

    if (count > Internals->NumberOfMarkers) {
        CMN_LOG_CLASS_RUN_WARNING << "Run: marker overflow, please increase number of markers.  Only the first "
                                  << Internals->NumberOfMarkers << " marker(s) will processed." << std::endl;
        count = Internals->NumberOfMarkers;
    }

    for (size_t m = 0; m < count; m++)
    {
        printf("geometry %u, trans (%.2f %.2f %.2f), error %.3f\n",
            Internals->Markers[m].geometryId,
            Internals->Markers[m].translationMM[0],
            Internals->Markers[m].translationMM[1],
            Internals->Markers[m].translationMM[2],
            Internals->Markers[m].registrationErrorMM);

        // Check for other marker fields to get rotation, fiducial id, etc.
    }

}


void mtsAtracsysFusionTrack::Cleanup(void)
{
    ftkClose(Internals->Library);
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
