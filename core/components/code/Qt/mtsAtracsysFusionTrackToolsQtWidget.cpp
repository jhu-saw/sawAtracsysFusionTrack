/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Juan Antonio Barragan
  Created on: 2026-11-09

  (C) Copyright 2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrackToolsQtWidget.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsManagerComponentServices.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidget.h>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

mtsAtracsysFusionTrackToolsQtWidget::mtsAtracsysFusionTrackToolsQtWidget(const std::string & componentName):
    QWidget(),
    mtsComponent(componentName),
    mNumberOfWidgets(0)
{
    this->AddTag("UI");
    mPrismaticFactor = cmn_mm;
    mRevoluteFactor = cmnPI_180;
    EnableDynamicComponentManagement();
    setupUi();
    startTimer(50); // ms
}

void mtsAtracsysFusionTrackToolsQtWidget::setupUi(void)
{
    mLayout = new QGridLayout();
    setLayout(mLayout);

    // connect event
    connect(this, SIGNAL(SignalCRTKInterfacesProvidedUpdated()),
            this, SLOT(SlotCRTKInterfacesProvidedUpdated()));
}

void mtsAtracsysFusionTrackToolsQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    if (this->isHidden()) {
        return;
    }
    prmPositionCartesianGet position;
    double registrationError;
    for (auto & source : mSources) {
        Source * s = source.second;
        s->measured_cp(position);
        s->mPositionWidget->SetValue(position);
        if (s->mHasRegistrationError) {
            if (s->registration_error(registrationError)) {
                s->mRegistrationErrorMM->setText(QString::number(registrationError / cmn_mm, 'f', 4));
            } else {
                s->mRegistrationErrorMM->setText("n/a");
            }
        }
    }
}

void mtsAtracsysFusionTrackToolsQtWidget::AddFactorySource(const std::string & componentName,
                                                           const std::string & interfaceName)
{
    mtsDescriptionInterfaceFullName source(mtsManagerLocal::GetInstance()->GetProcessName(),
                                           componentName, interfaceName);

    // first, make sure this doesn't already exist
    if (mFactories.find(source) != mFactories.end()) {
        CMN_LOG_CLASS_INIT_WARNING << "AddFactorySource: factory already added for component \""
                                   << componentName << "\" with interface \""
                                   << interfaceName << "\"" << std::endl;
        return;
    }
    // create new factory
    Factory * newFactory = new Factory;
    newFactory->mComponentWidget = this;
    mFactories[source] = newFactory;

    // create a new interface for the factory
    std::string reqInterfaceName = "_" + componentName + "_using_" + interfaceName + "_factory";
    mtsInterfaceRequired * required = AddInterfaceRequired(reqInterfaceName);
    if (required) {
        required->AddFunction("crtk_interfaces_provided", newFactory->mCRTKInterfacesProvided);
        required->AddEventHandlerVoid(&Factory::CRTKInterfacesProvidedUpdatedHandler,
                                      newFactory, "crtk_interfaces_provided_updated");
        mConnections.Add(this->GetName(), reqInterfaceName,
                         componentName, interfaceName);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddFactorySource: there's already an interface name \""
                                 << reqInterfaceName << "\"" << std::endl;
    }
}

void mtsAtracsysFusionTrackToolsQtWidget::Factory::CRTKInterfacesProvidedUpdatedHandler(void)
{
    // event handlers are not thread safe, emit so Qt can manage thread safety
    emit mComponentWidget->SignalCRTKInterfacesProvidedUpdated();
}

void mtsAtracsysFusionTrackToolsQtWidget::SlotCRTKInterfacesProvidedUpdated(void)
{
    for (auto & factory : mFactories) {
        std::vector<mtsDescriptionInterfaceFullName> possibleSources;
        factory.second->mCRTKInterfacesProvided(possibleSources);
        for (const auto & source : possibleSources) {
            // make sure this source is not already in use
            if (mSources.find(source) != mSources.end()) {
                continue;
            }
            // check that the source has a read command returning a cartesian position
            mtsInterfaceProvidedDescription interfaceDescription =
                ManagerComponentServices->GetInterfaceProvidedDescription(source);
            if (!mtsDescriptionHasName(interfaceDescription.CommandsRead,
                                       std::string("measured_cp"))) {
                continue;
            }
            const bool hasRegistrationError =
                mtsDescriptionHasName(interfaceDescription.CommandsRead,
                                      std::string("registration_error"));

            std::string interfaceName = source.ProcessName
                + "::" + source.ComponentName + "::" + source.InterfaceName;
            mtsInterfaceRequired * newInterface = AddInterfaceRequired(interfaceName);
            if (!newInterface) {
                CMN_LOG_CLASS_INIT_ERROR << "SlotCRTKInterfacesProvidedUpdated: "
                                         << this->GetName() << ", unable to create required interface \""
                                         << interfaceName << "\"" << std::endl;
                continue;
            }

            Source * newSource = new Source;
            newSource->mInterface = newInterface;
            newSource->mHasRegistrationError = hasRegistrationError;
            newInterface->AddFunction("measured_cp", newSource->measured_cp);
            if (hasRegistrationError) {
                newInterface->AddFunction("registration_error", newSource->registration_error);
            }

            // widgets for this tool
            QWidget * toolWidget = new QWidget;
            QVBoxLayout * toolLayout = new QVBoxLayout;
            toolLayout->setContentsMargins(0, 0, 0, 0);
            toolWidget->setLayout(toolLayout);

            newSource->mPositionWidget = new prmPositionCartesianGetQtWidget;
            newSource->mPositionWidget->SetPrismaticRevoluteFactors(mPrismaticFactor, mRevoluteFactor);
            toolLayout->addWidget(newSource->mPositionWidget);

            if (hasRegistrationError) {
                QGroupBox * errorBox = new QGroupBox("Registration error");
                QHBoxLayout * errorLayout = new QHBoxLayout;
                errorBox->setLayout(errorLayout);

                newSource->mRegistrationErrorMM = new QLabel("n/a");
                newSource->mRegistrationErrorMM->setTextInteractionFlags(Qt::TextSelectableByMouse);
                errorLayout->addWidget(newSource->mRegistrationErrorMM);
                errorLayout->addWidget(new QLabel("mm"));
                errorLayout->addStretch();

                toolLayout->addWidget(errorBox);
            }

            // grid position
            const int NB_COLS = 2;
            const int position = mNumberOfWidgets;
            mLayout->addWidget(toolWidget, position / NB_COLS, position % NB_COLS);
            mNumberOfWidgets++;

            mSources[source] = newSource;

            // request connection
            ManagerComponentServices->Connect(this->GetName(), interfaceName,
                                              source.ComponentName, source.InterfaceName);
        }
    }
}
