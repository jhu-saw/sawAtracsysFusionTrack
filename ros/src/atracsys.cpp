/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidgetFactory.h>
#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrack.h>
#include <sawAtracsysFusionTrack/mtsAtracsysStereo.h>
#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrackStrayMarkersQtWidget.h>

#include "mts_ros_crtk_atracsys_bridge.h"

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsAtracsysFusionTrack", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsAtracsysStereo", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    cisst_ral::ral ral(argc, argv, "atracsys");
    auto rosNode = ral.node();

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    double rosPeriod = 10.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;
    std::list<std::string> managerConfig;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the tracker component",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the arm component's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(ral.stripped_arguments(), std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsAtracsysFusionTrack * tracker = new mtsAtracsysFusionTrack("atracsys");
    tracker->Configure(jsonConfigFile);

    if (!tracker->HardwareInitialized()) {
        CMN_LOG_INIT_ERROR << "Failed to initialize Atracsys hardware --- is device connected and powered on?" << std::endl;
        return -1;
    }

    mtsAtracsysStereo * stereo = new mtsAtracsysStereo("stereo", tracker->GetName());
    stereo->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(tracker);
    componentManager->AddComponent(stereo);
    componentManager->Connect(tracker->GetName(), "StereoRaw",
                              stereo->GetName(), "StereoRaw");

    // ROS CRTK bridge
    mts_ros_crtk_atracsys_bridge * crtk_bridge
        = new mts_ros_crtk_atracsys_bridge("atracsys_crtk_bridge", rosNode);

    crtk_bridge->bridge(tracker->GetName(), "Controller", rosPeriod, tfPeriod);
    std::string stereo_namespace = tracker->GetName() + "/stereo";
    cisst_ral::clean_namespace(stereo_namespace);
    crtk_bridge->bridge_interface_provided(stereo->GetName(), "stereo", stereo_namespace, rosPeriod, tfPeriod);

    auto num_tools = tracker->GetNumberOfTools();
    for (size_t i = 0; i < num_tools; ++i) {
        crtk_bridge->bridge_tool_error(tracker->GetName(), tracker->GetToolName(i));
    }

    componentManager->AddComponent(crtk_bridge);
    crtk_bridge->Connect();

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // stray markers
    mtsAtracsysFusionTrackStrayMarkersQtWidget * strayMarkersWidget;
    strayMarkersWidget = new mtsAtracsysFusionTrackStrayMarkersQtWidget("StrayMarkers-GUI");
    strayMarkersWidget->Configure();
    componentManager->AddComponent(strayMarkersWidget);
    componentManager->Connect(strayMarkersWidget->GetName(), "Controller",
                              tracker->GetName(), "Controller");
    tabWidget->addTab(strayMarkersWidget, "Stray Markers");

    // tool position widgets
    prmPositionCartesianGetQtWidgetFactory * positionQtWidgetFactory
        = new prmPositionCartesianGetQtWidgetFactory("positionQtWidgetFactory");
    positionQtWidgetFactory->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI); // to display values in mm and degrees
    positionQtWidgetFactory->AddFactorySource(tracker->GetName(), "Controller");
    componentManager->AddComponent(positionQtWidgetFactory);
    positionQtWidgetFactory->Connect();
    tabWidget->addTab(positionQtWidgetFactory, "Tools");

    // custom user components
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    cisst_ral::shutdown();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
