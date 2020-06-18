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
#include <QString>
#include <QtGui>
#include <QMessageBox>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrackStrayMarkersQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAtracsysFusionTrackStrayMarkersQtWidget, mtsComponent, std::string);

mtsAtracsysFusionTrackStrayMarkersQtWidget::mtsAtracsysFusionTrackStrayMarkersQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetNumberOfThreeDFiducials", Controller.GetNumberOfThreeDFiducials);
        interfaceRequired->AddFunction("GetThreeDFiducialPosition", Controller.GetThreeDFiducialPosition);
        interfaceRequired->AddFunction("GetPeriodStatistics", Controller.GetPeriodStatistics);
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
    int numberOfMarkers;
    executionResult = Controller.GetNumberOfThreeDFiducials(numberOfMarkers);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Controller.GetNumberOfThreeDFiducials failed, \""
                                << executionResult << "\"" << std::endl;
    }
    else {
        QLNumberOfMarkers->setNum(numberOfMarkers);
    }

    Controller.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsAtracsysFusionTrackStrayMarkersQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    // Side by side for 3D position and timing
    QHBoxLayout * topLayout = new QHBoxLayout;
    mainLayout->addLayout(topLayout);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout();
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    topLayout->addLayout(timingLayout);

    // Vectors of values
    QGridLayout * gridLayout = new QGridLayout;
    mainLayout->addLayout(gridLayout);

    gridLayout->setSpacing(1);
    int row = 0;
    gridLayout->addWidget(new QLabel("Number of markers"), row, 0);
    QLNumberOfMarkers = new QLabel();
    gridLayout->addWidget(QLNumberOfMarkers, row, 1);
    row++;

    setLayout(mainLayout);
    setWindowTitle("StrayMarkers");
    resize(sizeHint());
}
