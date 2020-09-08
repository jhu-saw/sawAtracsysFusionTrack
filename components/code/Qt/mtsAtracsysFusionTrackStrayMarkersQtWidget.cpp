/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// Qt include
#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>

// cisst
#include <cisstVector/vctPose3DQtWidget.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionCartesianArrayGet.h>
#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrackStrayMarkersQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAtracsysFusionTrackStrayMarkersQtWidget, mtsComponent, std::string);

mtsAtracsysFusionTrackStrayMarkersQtWidget::mtsAtracsysFusionTrackStrayMarkersQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp_array_size", Controller.measured_cp_array_size);
        interfaceRequired->AddFunction("measured_cp_array", Controller.measured_cp_array);
        interfaceRequired->AddFunction("period_statistics", Controller.period_statistics);
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsAtracsysFusionTrackStrayMarkersQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsAtracsysFusionTrackStrayMarkersQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsAtracsysFusionTrackStrayMarkersQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsAtracsysFusionTrackStrayMarkersQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsAtracsysFusionTrackStrayMarkersQtWidget::Cleanup" << std::endl;
}

void mtsAtracsysFusionTrackStrayMarkersQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsAtracsysFusionTrackStrayMarkersQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsAtracsysFusionTrackStrayMarkersQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }
    mtsExecutionResult executionResult;
    size_t numberOfMarkers;
    executionResult = Controller.measured_cp_array_size(numberOfMarkers);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Controller.measured_cp_array_size failed, \""
                                << executionResult << "\"" << std::endl;
    }
    else {
        QLNumberOfMarkers->setNum(static_cast<int>(numberOfMarkers));
        prmPositionCartesianArrayGet poses;
        Controller.measured_cp_array(poses);
        QVPoses->Clear();
        for (const auto & pose : poses.Positions()) {
            QVPoses->SetValue(pose);
        }
    }

    Controller.period_statistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsAtracsysFusionTrackStrayMarkersQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    // Timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    mainLayout->addWidget(QMIntervalStatistics);

    // Vectors of values
    QHBoxLayout * nbPosesLayout = new QHBoxLayout;
    nbPosesLayout->addWidget(new QLabel("Number of markers"));
    QLNumberOfMarkers = new QLabel();
    nbPosesLayout->addWidget(QLNumberOfMarkers);
    mainLayout->addLayout(nbPosesLayout);

    // 3D display of markers
    QVPoses = new vctPose3DQtWidget();
    mainLayout->addWidget(QVPoses);

    setLayout(mainLayout);
    setWindowTitle("StrayMarkers");
    resize(sizeHint());
}
