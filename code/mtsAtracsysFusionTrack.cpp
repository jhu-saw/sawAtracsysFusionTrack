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

class mtsAtracsysFusionTrackTool
{
public:
    mtsAtracsysFusionTrackTool(const std::string & name) :
        Name(name),
        Interface(0),
        StateTable(500, name)
    {}

    ~mtsAtracsysFusionTrackTool(void) {}

    std::string Name;
    mtsInterfaceProvided * Interface;
    prmPositionCartesianGet Position;
    double RegistrationError;
    mtsStateTable StateTable;
};

class mtsAtracsysFusionTrackInternals
{
public:
	mtsAtracsysFusionTrackInternals(const size_t numberOfMarkers = 32) :
		NumberOfMarkers(numberOfMarkers),
        Library(0),
        Device(0)
    {
        memset(&Frame, 0, sizeof(ftkFrameQuery));
        Markers = new ftkMarker[NumberOfMarkers];
        Frame.markers = Markers;
        Frame.markersVersionSize.ReservedSize = sizeof(ftkMarker) * NumberOfMarkers;
        Frame.threeDFiducials = threedFiducials;
        Frame.threeDFiducialsVersionSize.ReservedSize = sizeof (threedFiducials);
        
    };

	size_t NumberOfMarkers;
    ftkLibrary Library;
    uint64 Device;
    ftkFrameQuery Frame;
	ftkMarker * Markers;
    ftk3DFiducial threedFiducials[100u];

    typedef std::map<uint32, mtsAtracsysFusionTrackTool *> GeometryIdToToolMap;
    GeometryIdToToolMap GeometryIdToTool;
};


void mtsAtracsysFusionTrackDeviceEnum(uint64 device, void * user, ftkDeviceType type)
{
    uint64 * lastDevice = reinterpret_cast<uint64 *>(user);
    if (lastDevice) {
        *lastDevice = device;
    }
}


void mtsAtracsysFusionTrack::Construct(void)
{
    Internals = new mtsAtracsysFusionTrackInternals();

	StateTable.AddData(NumberOfThreeDFiducials, "NumberOfThreeDFiducials");
	StateTable.AddData(ThreeDFiducialPosition, "ThreeDFiducialPosition");

    mtsInterfaceProvided * provided = AddInterfaceProvided("Controller");
    if (provided) {
//        provided->AddCommandReadState(StateTable, IsTracking, "IsTracking");
		provided->AddCommandReadState(StateTable, NumberOfThreeDFiducials, "GetNumberOfThreeDFiducials");
		provided->AddCommandReadState(StateTable, ThreeDFiducialPosition, "GetThreeDFiducialPosition");

    }
}


void mtsAtracsysFusionTrack::Configure(const std::string & filename)
{

	// initialize fusion track library
	Internals->Library = ftkInit();
	if (!Internals->Library) {
		CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to initialize (" << this->GetName() << ")" << std::endl;
		return;
	}

	// search for devices
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


void mtsAtracsysFusionTrack::Startup(void)
{
    std::cerr << "Startup" << std::endl;
}



void mtsAtracsysFusionTrack::Run(void)
{
	// process mts commands
	ProcessQueuedCommands();

	// get latest frame from fusion track library/device
	if (ftkGetLastFrame(Internals->Library,
		Internals->Device,
		&(Internals->Frame),
		0 ) != FTK_OK) {
		CMN_LOG_CLASS_RUN_DEBUG << "Run: timeout on ftkGetLastFrame" << std::endl;
		return;
	}

	// check results of last frame
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

	// make sure we're not getting more markers than allocated
	size_t count = Internals->Frame.markersCount;
	if (count > Internals->NumberOfMarkers) {
		CMN_LOG_CLASS_RUN_WARNING << "Run: marker overflow, please increase number of markers.  Only the first "
			<< Internals->NumberOfMarkers << " marker(s) will processed." << std::endl;
		count = Internals->NumberOfMarkers;
	}

	// initialize all tools
	const ToolsType::iterator end = Tools.end();
	ToolsType::iterator iter;
	for (iter = Tools.begin(); iter != end; ++iter) {
		iter->second->StateTable.Start();
		iter->second->Position.SetValid(false);
	}

	// for each marker, get the data and populate corresponding tool
	for (size_t index = 0; index < count; ++index) {
		ftkMarker * currentMarker = &(Internals->Markers[index]);
		// find the appropriate tool
		if (Internals->GeometryIdToTool.find(currentMarker->geometryId) == Internals->GeometryIdToTool.end()) {
			CMN_LOG_CLASS_RUN_WARNING << "Run: found a geometry Id not registered using AddTool, this marker will be ignored ("
				<< this->GetName() << ")" << std::endl;
		}
		else {
			mtsAtracsysFusionTrackTool * tool = Internals->GeometryIdToTool.at(currentMarker->geometryId);
			tool->Position.SetValid(true);
			tool->Position.Position().Translation().Assign(currentMarker->translationMM[0],
				currentMarker->translationMM[1],
				currentMarker->translationMM[2]);
			for (size_t row = 0; row < 3; ++row) {
				for (size_t col = 0; col < 3; ++col) {
					tool->Position.Position().Rotation().Element(row, col) = currentMarker->rotation[row][col];
				}
			}

			tool->RegistrationError = currentMarker->registrationErrorMM;
			
			printf("Marker:\n");
			printf("XYZ (%.2f %.2f %.2f)\n",
					currentMarker->translationMM[0],
					currentMarker->translationMM[1],
					currentMarker->translationMM[2]);
		}
	}

	// finalize all tools
	for (iter = Tools.begin(); iter != end; ++iter) {
		iter->second->StateTable.Advance();
	}

	// ---- 3D Fiducials ---
	switch (Internals->Frame.threeDFiducialsStat)
	{
	case QS_WAR_SKIPPED:
		CMN_LOG_CLASS_RUN_ERROR << "Run: 3D status fields in the frame is not set correctly" << std::endl;
	case QS_ERR_INVALID_RESERVED_SIZE:
		CMN_LOG_CLASS_RUN_ERROR << "Run: frame.threeDFiducialsVersionSize is invalid" << std::endl;
	default:
		CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
	case QS_OK:
		break;
	}

	ThreeDFiducialPosition.clear();
	NumberOfThreeDFiducials = Internals->Frame.threeDFiducialsCount;
	ThreeDFiducialPosition.resize(NumberOfThreeDFiducials);


	printf("3D fiducials:\n");
	for (uint32 m = 0; m < NumberOfThreeDFiducials; m++)
	{
		printf("\tINDEXES (%u %u)\t XYZ (%.2f %.2f %.2f)\n\t\tEPI_ERR: %.2f\tTRI_ERR: %.2f\tPROB: %.2f\n",
			Internals->threedFiducials[m].leftIndex,
			Internals->threedFiducials[m].rightIndex,
			Internals->threedFiducials[m].positionMM.x,
			Internals->threedFiducials[m].positionMM.y,
			Internals->threedFiducials[m].positionMM.z,
			Internals->threedFiducials[m].epipolarErrorPixels,
			Internals->threedFiducials[m].triangulationErrorMM,
			Internals->threedFiducials[m].probability);

		ThreeDFiducialPosition[m].X() = Internals->threedFiducials[m].positionMM.x;
		ThreeDFiducialPosition[m].Y() = Internals->threedFiducials[m].positionMM.y;
		ThreeDFiducialPosition[m].Z() = Internals->threedFiducials[m].positionMM.z;
	}

}

void mtsAtracsysFusionTrack::Cleanup(void)
{
    ftkClose(Internals->Library);
}


bool mtsAtracsysFusionTrack::AddToolIni(const std::string & toolName, const std::string & fileName)
{

    // check if this tool already exists
    mtsAtracsysFusionTrackTool * tool = Tools.GetItem(toolName);
    if (tool) {
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: " << tool->Name << " already exists" << std::endl;
        return false;
    }

    // make sure we can find and load this tool ini file
    ftkError error;
    ftkGeometry geometry;
    switch (loadGeometry(Internals->Library, Internals->Device, fileName, geometry))
    {
    case 1:
        CMN_LOG_CLASS_INIT_VERBOSE << "AddToolIni: loaded " << fileName << " from installation directory" << std::endl;
    case 0:
        error = ftkSetGeometry(Internals->Library, Internals->Device, &geometry);
        if (error != FTK_OK) {
            CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: unable to set geometry for tool " << fileName << " (" << this->GetName() << ")" << std::endl;
            return false;
        }
        break;
    default:
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: error, cannot load geometry file " << fileName << std::endl;
        return false;
    }

    // make sure there is no such geometry Id yet
    const mtsAtracsysFusionTrackInternals::GeometryIdToToolMap::const_iterator
        toolIterator = Internals->GeometryIdToTool.find(geometry.geometryId);
    if (toolIterator != Internals->GeometryIdToTool.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: error, found an existing tool with the same Id " << geometry.geometryId << " for " << fileName << std::endl;
        return false;
    }

    // finally create a cisst tool structure
    tool = new mtsAtracsysFusionTrackTool(toolName);

    // create an interface for tool
    tool->Interface = AddInterfaceProvided(toolName);
    if (!tool->Interface) {
        CMN_LOG_CLASS_INIT_ERROR << "AddToolIni: " << tool->Name << " already exists" << std::endl;
        delete tool;
        return false;
    }

    // regiter newly created tool
    this->Tools.AddItem(toolName, tool);
    Internals->GeometryIdToTool[geometry.geometryId] = tool;

    // add data for this tool and populate tool interface
    tool->StateTable.SetAutomaticAdvance(false);
    this->AddStateTable(&(tool->StateTable));
    tool->StateTable.AddData(tool->Position, "Position");
    tool->StateTable.AddData(tool->RegistrationError, "RegistrationError");
    tool->Interface->AddCommandReadState(tool->StateTable, tool->Position, "GetPositionCartesian");
    tool->Interface->AddCommandReadState(tool->StateTable, tool->RegistrationError, "GetRegistrationError");
    tool->Interface->AddCommandReadState(tool->StateTable,
                                         tool->StateTable.PeriodStats,
                                         "GetPeriodStatistics");

    return true;
}


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