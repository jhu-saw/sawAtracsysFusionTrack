/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \file
  \brief An example interface for NDI trackers with serial interface.
  \ingroup devicesTutorial
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsCollectorState.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrack.h>
#include "sawAtracsysFusionTrackDataLinker.h"


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsAtracsysFusionTrack", CMN_LOG_ALLOW_ALL);
	cmnLogger::SetMaskClassMatching("sawAtracsysFusionTrackDataLinker", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create the components
    mtsAtracsysFusionTrack * tracker = new mtsAtracsysFusionTrack("FusionTrack");
    // tracker->Configure("/home/rems/dev/cisst-saw/sawAtracsysFusionTrack/examples/configAtracsysFusionTrack.json");
    tracker->Configure("");
    tracker->AddToolIni("Tip Tool", "geometry_tool.ini");
    tracker->AddToolIni("Arm Tool", "geometry_arm_recalibrated.ini");
    tracker->AddToolIni("Reference", "geometry_ref.ini");

	// add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(tracker);

	sawAtracsysFusionTrackDataLinker * dataLinker = new sawAtracsysFusionTrackDataLinker("dataLinker", 5.0 * cmn_ms);
	componentManager->AddComponent(dataLinker);

	componentManager->Connect(dataLinker->GetName(), "Controller",
							  tracker->GetName(), "Controller");

	componentManager->Connect(dataLinker->GetName(), "Tip Tool",
								tracker->GetName(), "Tip Tool");
    componentManager->Connect(dataLinker->GetName(), "Arm Tool",
                            tracker->GetName(), "Arm Tool");
    componentManager->Connect(dataLinker->GetName(), "Reference",
                        tracker->GetName(), "Reference");

    
    // create and start all components
    componentManager->CreateAll();
    componentManager->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);
    componentManager->StartAll();
    componentManager->WaitForStateAll(mtsComponentState::ACTIVE, 2.0 * cmn_s);
    
    int ch;
    bool started = false;
    while (ch != 'q') {
        osaSleep(1.0 * cmn_s);
    }
    
    // kill all components and perform cleanup
    componentManager->KillAll();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);
    componentManager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
