/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _sawAtracsysFusionTrackDataLinker_h
#define _sawAtracsysFusionTrackDataLinker_h

// cisst include
#include <cisstCommon/cmnClassServices.h>
#include <cisstMultiTask.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstVector.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
//#include <sawAtracsysFusionTrack\mtsAtracsysFusionTrack.h>


#undef CISST_EXPORT
#define CISST_EXPORT


// The main task.
class sawAtracsysFusionTrackDataLinker : public mtsTaskPeriodic {
	CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
public:
    sawAtracsysFusionTrackDataLinker(const std::string & name, double periodInSeconds);
    ~sawAtracsysFusionTrackDataLinker() {}
    virtual void Run(void);
    virtual void Cleanup(void);
    virtual void Startup(void);
    virtual void Configure(const char *filename);

    prmPositionCartesianGet prmTool;

    unsigned int BatchReadyEventCounter; // counter for range events from state table
    mtsStateTable::IndexRange LastRange;
    bool CollectionRunning;
    unsigned int SamplesCollected;

protected:

	struct {
		mtsFunctionVoid Start;
		mtsFunctionVoid Stop;
	} Collector;

	struct ToolStruct {
		mtsFunctionRead GetPositionCartesian;
		mtsFunctionRead GetRegistrationError;
		mtsFunctionRead GetPeriodStatistics;
	};

	struct FiducialStruct {
		mtsFunctionRead GetNumberOfThreeDFiducials;
		mtsFunctionRead GetThreeDFiducialPosition;

		int NumberOfThreeDFiducials;
		std::vector<vct3> ThreeDFiducialPosition;
	};


	ToolStruct Tool;
	ToolStruct Arm;
	ToolStruct Ref;

	FiducialStruct ThreeDFiducials;

    void AddToolInterface(const std::string & toolName, ToolStruct & functionSet);
};

CMN_DECLARE_SERVICES_INSTANTIATION(sawAtracsysFusionTrackDataLinker)

#endif // _sawAtracsysFusionTrackDataLinker_h
