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

#ifndef _mtsAtracsysFusionTrackToolsQtWidget_h
#define _mtsAtracsysFusionTrackToolsQtWidget_h

#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsDelayedConnections.h>

#include <QWidget>

// Always include last
#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackQtExport.h>

class QGridLayout;
class QLabel;
class prmPositionCartesianGetQtWidget;

/*! Widget displaying, for each tool reported by the tracker
  (crtk_interfaces_provided), the measured_cp and the registration
  error in millimeters. */
class CISST_EXPORT mtsAtracsysFusionTrackToolsQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;

public:
    mtsAtracsysFusionTrackToolsQtWidget(const std::string & componentName);
    ~mtsAtracsysFusionTrackToolsQtWidget(void) {};

    inline void SetPrismaticRevoluteFactors(const double & prismatic, const double & revolute) {
        mPrismaticFactor = prismatic;
        mRevoluteFactor = revolute;
    }

    void AddFactorySource(const std::string & componentName,
                          const std::string & interfaceName);

    inline virtual void Connect(void) {
        mConnections.Connect();
    }

protected:
    void setupUi(void);
    void timerEvent(QTimerEvent * event);

    double mPrismaticFactor, mRevoluteFactor;

    mtsDelayedConnections mConnections;

    class Factory {
    public:
        mtsAtracsysFusionTrackToolsQtWidget * mComponentWidget;
        mtsFunctionRead mCRTKInterfacesProvided;
        void CRTKInterfacesProvidedUpdatedHandler(void);
    };

    typedef std::map<mtsDescriptionInterfaceFullName, Factory *> FactoriesType;
    FactoriesType mFactories;

    class Source {
    public:
        mtsInterfaceRequired * mInterface = nullptr;
        mtsFunctionRead measured_cp;
        mtsFunctionRead registration_error;
        bool mHasRegistrationError = false;
        prmPositionCartesianGetQtWidget * mPositionWidget = nullptr;
        QLabel * mRegistrationErrorMM = nullptr;
    };

    typedef std::map<mtsDescriptionInterfaceFullName, Source *> SourcesType;
    SourcesType mSources;

    // Qt stuff
    int mNumberOfWidgets;
    QGridLayout * mLayout;

signals:
    void SignalCRTKInterfacesProvidedUpdated(void);

protected slots:
    void SlotCRTKInterfacesProvidedUpdated(void);
};

#endif // _mtsAtracsysFusionTrackToolsQtWidget_h
