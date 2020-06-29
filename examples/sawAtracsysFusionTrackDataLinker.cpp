/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

// system include 
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

// cisst include
#include <cisstCommon/cmnPortability.h>
#include <cisstOSAbstraction/osaSleep.h>
#include "sawAtracsysFusionTrackDataLinker.h"

CMN_IMPLEMENT_SERVICES(sawAtracsysFusionTrackDataLinker);

sawAtracsysFusionTrackDataLinker::sawAtracsysFusionTrackDataLinker(const std::string & name, double periodInSeconds) :
    mtsTaskPeriodic(name, periodInSeconds),
    BatchReadyEventCounter(0),
    CollectionRunning(false),
    SamplesCollected(0)
{
	mtsInterfaceRequired * required = AddInterfaceRequired("Controller", MTS_OPTIONAL);
	if (required) {
		required->AddFunction("GetNumberOfThreeDFiducials", ThreeDFiducials.GetNumberOfThreeDFiducials);
		required->AddFunction("GetThreeDFiducialPosition", ThreeDFiducials.GetThreeDFiducialPosition);
	}

	required = AddInterfaceRequired("DataCollector", MTS_OPTIONAL);
    if (required) {
        required->AddFunction("StartCollection", Collector.Start);
        required->AddFunction("StopCollection", Collector.Stop);
    }
    
    // the following two tools are using names normally defined in configure file
	AddToolInterface("Tip Tool", Tool);
	AddToolInterface("Arm Tool", Arm);
	AddToolInterface("Reference", Ref);
}

void sawAtracsysFusionTrackDataLinker::AddToolInterface(const std::string & toolName,
                                                               sawAtracsysFusionTrackDataLinker::ToolStruct & functionSet)
{
	mtsInterfaceRequired * required = AddInterfaceRequired(toolName, MTS_OPTIONAL);
    if (required) {
		required->AddFunction("measured_cp", functionSet.measured_cp);
		required->AddFunction("GetRegistrationError", functionSet.GetRegistrationError);
		required->AddFunction("period_statistics", functionSet.period_statistics);
    }
}

void sawAtracsysFusionTrackDataLinker::Configure(const char *filename) {
    
}

void sawAtracsysFusionTrackDataLinker::Startup(void){	
}
void sawAtracsysFusionTrackDataLinker::Run(void){
	ProcessQueuedEvents();
    ProcessQueuedCommands();

	Tool.measured_cp(prmTool);
	
	if (!prmTool.Valid()) {
		CMN_LOG_CLASS_RUN_ERROR << "Tool.measured_cp failed"
			<< std::endl;
	}
	else {
		std::cout << "Tool from Atracsys Fusion Tracker (Translation): " << prmTool.Position().Translation() << std::endl;
		std::cout << "Tool from Atracsys Fusion Tracker (Orientation): " << std::endl << prmTool.Position().Rotation() << std::endl;
	}

	Arm.measured_cp(prmTool);
	
	if (!prmTool.Valid()) {
		CMN_LOG_CLASS_RUN_ERROR << "Arm.measured_cp failed"
			<< std::endl;
	}
	else {
		std::cout << "Arm from Atracsys Fusion Tracker (Translation): " << prmTool.Position().Translation() << std::endl;
		std::cout << "Arm from Atracsys Fusion Tracker (Orientation): " << std::endl << prmTool.Position().Rotation() << std::endl;
	}

	Ref.measured_cp(prmTool);
	
	if (!prmTool.Valid()) {
		CMN_LOG_CLASS_RUN_ERROR << "Ref.measured_cp failed"
			<< std::endl;
	}
	else {
		std::cout << "Ref from Atracsys Fusion Tracker (Translation): " << prmTool.Position().Translation() << std::endl;
		std::cout << "Ref from Atracsys Fusion Tracker (Orientation): " << std::endl << prmTool.Position().Rotation() << std::endl;
	}

	ThreeDFiducials.GetNumberOfThreeDFiducials(ThreeDFiducials.NumberOfThreeDFiducials);

	if (ThreeDFiducials.NumberOfThreeDFiducials != 0)
	{
		ThreeDFiducials.ThreeDFiducialPosition.resize(ThreeDFiducials.NumberOfThreeDFiducials);

		ThreeDFiducials.GetThreeDFiducialPosition(ThreeDFiducials.ThreeDFiducialPosition);

		for (unsigned int i = 0; i < ThreeDFiducials.NumberOfThreeDFiducials; i++)
		{
			std::cout << "ThreeDFiducialPosition[" << i << "]: " << ThreeDFiducials.ThreeDFiducialPosition[i].X() << "\t"
				<< ThreeDFiducials.ThreeDFiducialPosition[i].Y() << "\t" << ThreeDFiducials.ThreeDFiducialPosition[i].Z() << std::endl;
		}
	}
}
void sawAtracsysFusionTrackDataLinker::Cleanup(void) {
	
}
