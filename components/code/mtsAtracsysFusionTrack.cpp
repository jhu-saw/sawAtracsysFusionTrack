/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <ftkInterface.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackRevision.h>
#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrack.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAtracsysFusionTrack, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

bool LoadGeometryJSON(const std::string & filename,
                      ftkGeometry & geometry)
{
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Reader jsonReader;
    Json::Value jsonConfig;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_INIT_ERROR << "LoadGeometryJSON: failed to parse configuration \""
                           << filename << "\"" << std::endl
                           << jsonReader.getFormattedErrorMessages();
        return false;
    }

    // version?
    geometry.version = 0u;
    Json::Value jsonValue;

    // id
    jsonValue = jsonConfig["id"];
    if (!jsonValue.empty()) {
        geometry.geometryId = jsonValue.asUInt();
    } else {
        CMN_LOG_INIT_ERROR << "LoadGeometryJSON: missing \"id\" in \"" << std::endl
                           << filename << "\"" << std::endl;
        return false;
    }

    // id
    jsonValue = jsonConfig["count"];
    if (!jsonValue.empty()) {
        geometry.pointsCount = jsonValue.asUInt();
    } else {
        CMN_LOG_INIT_ERROR << "LoadGeometryJSON: missing \"count\" in \"" << std::endl
                           << filename << "\"" << std::endl;
        return false;
    }

    // fiducials
    const Json::Value jsonFiducials = jsonConfig["fiducials"];
    // check size
    if (jsonFiducials.size() != geometry.pointsCount) {
        CMN_LOG_INIT_ERROR << "LoadGeometryJSON: found " << jsonFiducials.size()
                           << " fiducial defined but expected " << geometry.pointsCount
                           << " based on \"count\" defined in \"" << std::endl
                           << filename << "\"" << std::endl;
        return false;
    }
    // load fiducial
    for (unsigned int index = 0; index < jsonFiducials.size(); ++index) {
        jsonValue = jsonFiducials[index];
        if (jsonValue["x"].empty()) {
            CMN_LOG_INIT_ERROR << "LoadGeometryJSON: \"x\" is missing for fiducial[" << index
                               << "] in \"" << std::endl
                               << filename << "\"" << std::endl;
            return false;
        } else {
            geometry.positions[index].x = jsonValue["x"].asDouble();
        }
        if (jsonValue["y"].empty()) {
            CMN_LOG_INIT_ERROR << "LoadGeometryJSON: \"y\" is missing for fiducial[" << index
                               << "] in \"" << std::endl
                               << filename << "\"" << std::endl;
            return false;
        } else {
            geometry.positions[index].y = jsonValue["y"].asDouble();
        }
        if (jsonValue["z"].empty()) {
            CMN_LOG_INIT_ERROR << "LoadGeometryJSON: \"z\" is missing for fiducial[" << index
                               << "] in \"" << std::endl
                               << filename << "\"" << std::endl;
            return false;
        } else {
            geometry.positions[index].z = jsonValue["z"].asDouble();
        }
    }
    return true;
}

#if sawAtracsysFusionTrack_USES_INI_PARSER
#include <iniparser/iniparser.h>
#endif

bool LoadGeometryINI(const std::string & filename,
                     ftkGeometry & geometry)
{
#if sawAtracsysFusionTrack_USES_INI_PARSER
    dictionary * ini = iniparser_load(filename.c_str());
    if (ini == 0) {
        CMN_LOG_INIT_ERROR << "LoadGeometryINI: failed to parse configuration \""
                           << filename << "\"" << std::endl;
        return false;
    }

    // parser data
    int i;
    double d;

    // version?
    geometry.version = 0u;

    // id
    i = iniparser_getint(ini, "geometry:id", -1);
    if (i == -1) {
        CMN_LOG_INIT_ERROR << "LoadGeometryINI: missing \"id\" in \"" << std::endl
                           << filename << "\"" << std::endl;
        return false;
    }
    geometry.geometryId = i;

    // count
    i = iniparser_getint(ini, "geometry:count", -1);
    if (i == -1) {
        CMN_LOG_INIT_ERROR << "LoadGeometryINI: missing \"count\" in \"" << std::endl
                           << filename << "\"" << std::endl;
        return false;
    }
    geometry.pointsCount = i;

    // fiducials
    const double notFound = -9.87654321;
    char entry[13]; // for fiducialN:x\0
    for (unsigned int index = 0; index < geometry.pointsCount; ++index) {
        // x
        sprintf(entry, "fiducial%u:x", index);
        d = iniparser_getdouble(ini, entry, notFound);
        if (d == notFound) {
            CMN_LOG_INIT_ERROR << "LoadGeometryINI: \"x\" is missing for fiducial[" << index
                               << "] in \"" << std::endl
                               << filename << "\"" << std::endl;
            return false;
        } else {
            geometry.positions[index].x = d;
        }
        // y
        sprintf(entry, "fiducial%u:y", index);
        d = iniparser_getdouble(ini, entry, notFound);
        if (d == notFound) {
            CMN_LOG_INIT_ERROR << "LoadGeometryINI: \"y\" is missing for fiducial[" << index
                               << "] in \"" << std::endl
                               << filename << "\"" << std::endl;
            return false;
        } else {
            geometry.positions[index].y = d;
        }
        // z
        sprintf(entry, "fiducial%u:z", index);
        d = iniparser_getdouble(ini, entry, notFound);
        if (d == notFound) {
            CMN_LOG_INIT_ERROR << "LoadGeometryINI: \"z\" is missing for fiducial[" << index
                               << "] in \"" << std::endl
                               << filename << "\"" << std::endl;
            return false;
        } else {
            geometry.positions[index].z = d;
        }
    }

    iniparser_freedict(ini);

    return true;
#else
    std::cerr << CMN_LOG_DETAILS << " sorry, this code was built without a .ini parser for tool geometry.  Convert your geometry file to JSON or re-compile this code with inih (a simple ini parsers)." << std::endl;
    return false;
#endif
}


class mtsAtracsysFusionTrackTool
{
public:
    mtsAtracsysFusionTrackTool(const std::string & name) :
        m_name(name),
        m_interface(0),
        m_state_table(500, name)
    {}

    ~mtsAtracsysFusionTrackTool(void) {}

    std::string m_name;
    mtsInterfaceProvided * m_interface;
    prmPositionCartesianGet m_measured_cp;
    double m_registration_error;
    mtsStateTable m_state_table;
};

class mtsAtracsysFusionTrackInternals
{
public:
    mtsAtracsysFusionTrackInternals(const size_t numberOfMarkers = 2) :
        NumberOfMarkers(numberOfMarkers),
        Library(0),
        Device(0LL)
    {
        Frame = ftkCreateFrame();
        memset(Frame, 0, sizeof(ftkFrameQuery));
        ftkError err(ftkSetFrameOptions(false, false, 16u, 16u, 7u, 2u,
                                        Frame));
        if (err != FTK_OK) {
            CMN_LOG_INIT_ERROR << "mtsAtracsysFusionTrackInternals: ftkSetFrameOptions failed" << std::endl;
        } else {
            CMN_LOG_INIT_VERBOSE << "mtsAtracsysFusionTrackInternals: ftkSetFrameOptions ok" << std::endl;
        }

        Markers = new ftkMarker[NumberOfMarkers];
        Frame->markers = Markers;
        Frame->markersVersionSize.ReservedSize = sizeof(ftkMarker) * NumberOfMarkers;
        Frame->threeDFiducials = threedFiducials;
        Frame->threeDFiducialsVersionSize.ReservedSize = sizeof(threedFiducials);
    };

    size_t NumberOfMarkers;
    ftkLibrary Library;
    uint64 Device;
    ftkFrameQuery * Frame;
    ftkMarker * Markers;
    ftk3DFiducial threedFiducials[7u];
    bool Configured;

    typedef std::map<uint32, mtsAtracsysFusionTrackTool *> GeometryIdToToolMap;
    GeometryIdToToolMap GeometryIdToTool;
};


void mtsAtracsysFusionTrackDeviceEnum(uint64 device, void * user, ftkDeviceType CMN_UNUSED(type))
{
    uint64 * lastDevice = reinterpret_cast<uint64 *>(user);
    if (lastDevice) {
        *lastDevice = device;
    }
}


void mtsAtracsysFusionTrack::Init(void)
{
    m_internals = new mtsAtracsysFusionTrackInternals();

    // configuration state table
    m_configuration_state_table.SetAutomaticAdvance(false);
    AddStateTable(&m_configuration_state_table);
    m_configuration_state_table.AddData(m_crtk_interfaces_provided, "crtk_interfaces_provided");

    StateTable.AddData(m_measured_cp_array_size, "measured_cp_array_size");
    StateTable.AddData(m_measured_cp_array, "measured_cp_array");

    mtsInterfaceProvided * provided = AddInterfaceProvided("Controller");
    if (provided) {
        // system info
        provided->AddCommandReadState(StateTable, m_measured_cp_array_size, "measured_cp_array_size");
        provided->AddCommandReadState(StateTable, m_measured_cp_array, "measured_cp_array");
        provided->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");

        // crtk interfaces
        provided->AddCommandReadState(m_configuration_state_table, m_crtk_interfaces_provided,
                                      "crtk_interfaces_provided");
        provided->AddEventVoid(m_crtk_interfaces_provided_updated, "crtk_interfaces_provided_updated");
    }
}


void mtsAtracsysFusionTrack::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    // initialize fusion track library
    m_internals->Library = ftkInit();
    if (!m_internals->Library) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to initialize ("
                                 << this->GetName() << ")" << std::endl;
        return;
    }

    for (size_t i = 0; i < 10; i++) {
        // search for devices
        ftkError error = ftkEnumerateDevices(m_internals->Library,
                                             mtsAtracsysFusionTrackDeviceEnum,
                                             &(m_internals->Device));
        if (error != FTK_OK) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to enumerate devices ("
                                     << this->GetName() << ")" << std::endl;
            ftkClose(&m_internals->Library);
        }

        if (m_internals->Device != 0LL) {
            break;
        }
    }
    if (m_internals->Device == 0LL) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: no device connected ("
                                 << this->GetName() << ")" << std::endl;
        ftkClose(&m_internals->Library);
        return;
    }

    if (filename == "") {
        return;
    }

    // read JSON file passed as param, see configAtracsysFusionTrack.json for an example
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        m_internals->Configured = false;
        return;
    }

    // allows the use of relative paths for geometry files
    cmnPath configPath(cmnPath::GetWorkingDirectory());

    // add FTK path too
    ftkBuffer buffer;
    if ((ftkGetData(m_internals->Library, m_internals->Device,
                    FTK_OPT_DATA_DIR, &buffer ) == FTK_OK) && (buffer.size > 0)) {
        std::string ftkPath(reinterpret_cast<char*>(buffer.data));
        configPath.Add(ftkPath);
    }

    // now, find the tool geometry, either as ini or json file
    const Json::Value jsonTools = jsonConfig["tools"];
    for (unsigned int index = 0; index < jsonTools.size(); ++index) {
        jsonValue = jsonTools[index];
        std::string toolName = jsonValue["name"].asString();

        // make sure toolName is valid
        bool isJson = false;
        std::string toolFile = "undefined";
        if (toolName == "") {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool name found in " << filename << std::endl;
        } else {
            const Json::Value iniTool = jsonValue["ini-file"];
            const Json::Value jsonTool = jsonValue["json-file"];
            if (iniTool.empty() && jsonTool.empty()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: you need to define either \"ini-file\" or \"json-file\" for the tool \""
                                         << toolName << "\" in configuration file \""
                                         << filename << "\", neither was found" << std::endl;
            } else if (!iniTool.empty() && !jsonTool.empty()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: you need to define either \"ini-file\" or \"json-file\" for the tool \""
                                         << toolName << "\" in configuration file \""
                                         << filename << "\", both were found" << std::endl;
            } else if (!iniTool.empty()) {
                isJson = false;
                toolFile = iniTool.asString();
            } else {
                isJson = true;
                toolFile = jsonTool.asString();
            }

            std::string fullname = configPath.Find(toolFile);
            // make sure ini file is valid
            if (cmnPath::Exists(fullname)) {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: calling AddTool with tool name: " << toolName << " and configuration file: " << filename << std::endl;
                AddTool(toolName, fullname, isJson);
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: configuration file \"" << toolFile
                                         << "\" for tool \"" << toolName
                                         << "\" not found in path (" << configPath << ")" << std::endl;
            }
        }
    }
}


void mtsAtracsysFusionTrack::Startup(void)
{
    CMN_LOG_CLASS_RUN_ERROR << "Startup" << std::endl;
    // trigger event so connected component can bridge crtk provided interface as needed
    m_configuration_state_table.Start();
    m_crtk_interfaces_provided.push_back(mtsDescriptionInterfaceFullName("localhost", this->Name, "Controller"));
    m_configuration_state_table.Advance();
    m_crtk_interfaces_provided_updated();

    // set reference frame for measured_cp_array
    m_measured_cp_array.ReferenceFrame() = this->Name;
}


void mtsAtracsysFusionTrack::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    // get latest frame from fusion track library/device
    ftkError status = ftkGetLastFrame(m_internals->Library,
                                      m_internals->Device,
                                      m_internals->Frame,
                                      100u);
    // negative error codes are warnings
    if (status != FTK_OK) {
        if (status < 0) {
            m_measured_cp_array.SetValid(true);
            // std::cerr << "Warning: " << status << std::endl;
        } else {
            m_measured_cp_array.SetValid(false);
            std::cerr << "Error: " << status << std::endl;
            return;
        }
    }

    // check results of last frame
    switch (m_internals->Frame->markersStat) {
    case QS_WAR_SKIPPED:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: marker fields in the frame are not set correctly" << std::endl;
        break;
    case QS_ERR_INVALID_RESERVED_SIZE:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: frame.markersVersionSize is invalid" << std::endl;
        break;
    default:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
        break;
    case QS_OK:
        break;
    }

    // make sure we're not getting more markers than allocated
    size_t count = m_internals->Frame->markersCount;
    if (count > m_internals->NumberOfMarkers) {
        CMN_LOG_CLASS_RUN_WARNING << "Run: marker overflow, please increase number of markers.  Only the first "
                                  << m_internals->NumberOfMarkers << " marker(s) will processed." << std::endl;
        count = m_internals->NumberOfMarkers;
    }

    // initialize all tools
    const ToolsType::iterator end = m_tools.end();
    ToolsType::iterator iter;
    for (iter = m_tools.begin(); iter != end; ++iter) {
        iter->second->m_state_table.Start();
        iter->second->m_measured_cp.SetValid(false);
    }

    // for each marker, get the data and populate corresponding tool
    for (size_t index = 0; index < count; ++index) {
        ftkMarker * currentMarker = &(m_internals->Markers[index]);

        // find the appropriate tool
        if (m_internals->GeometryIdToTool.find(currentMarker->geometryId) == m_internals->GeometryIdToTool.end()) {
            CMN_LOG_CLASS_RUN_WARNING << "Run: found a geometry Id not registered using AddTool, this marker will be ignored ("
                                      << this->GetName() << ")" << std::endl;
        }
        else {
            mtsAtracsysFusionTrackTool * tool = m_internals->GeometryIdToTool.at(currentMarker->geometryId);
            tool->m_measured_cp.SetValid(true);
            tool->m_measured_cp.Position().Translation().Assign(currentMarker->translationMM[0],
                                                                currentMarker->translationMM[1],
                                                                currentMarker->translationMM[2]);
            for (size_t row = 0; row < 3; ++row) {
                for (size_t col = 0; col < 3; ++col) {
                    tool->m_measured_cp.Position().Rotation().Element(row, col) = currentMarker->rotation[row][col];
                }
            }

            tool->m_registration_error = currentMarker->registrationErrorMM;

            /*
              printf("Marker:\n");
              printf("XYZ (%.2f %.2f %.2f)\n",
              currentMarker->translationMM[0],
              currentMarker->translationMM[1],
              currentMarker->translationMM[2]);
            */
        }
    }

    // finalize all tools
    for (iter = m_tools.begin(); iter != end; ++iter) {
        iter->second->m_state_table.Advance();
    }

    // ---- 3D Fiducials, aka stray markers ---
    switch (m_internals->Frame->threeDFiducialsStat) {
    case QS_WAR_SKIPPED:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: 3D status fields in the frame is not set correctly" << std::endl;
        break;
    case QS_ERR_INVALID_RESERVED_SIZE:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: frame.threeDFiducialsVersionSize is invalid" << std::endl;
        break;
    default:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
        break;
    case QS_OK:
        break;
    }

    m_measured_cp_array_size = m_internals->Frame->threeDFiducialsCount;
    m_measured_cp_array.Positions().resize(m_measured_cp_array_size);

    //printf("3D fiducials:\n");
    for (uint32 m = 0; m < m_measured_cp_array_size; m++) {
        /*
          printf("\tINDEXES (%u %u)\t XYZ (%.2f %.2f %.2f)\n\t\tEPI_ERR: %.2f\tTRI_ERR: %.2f\tPROB: %.2f\n",
          m_internals->threedFiducials[m].leftIndex,
          m_internals->threedFiducials[m].rightIndex,
          m_internals->threedFiducials[m].positionMM.x,
          m_internals->threedFiducials[m].positionMM.y,
          m_internals->threedFiducials[m].positionMM.z,
          m_internals->threedFiducials[m].epipolarErrorPixels,
          m_internals->threedFiducials[m].triangulationErrorMM,
          m_internals->threedFiducials[m].probability);
        */
        m_measured_cp_array.Positions().at(m).Translation().Assign(m_internals->threedFiducials[m].positionMM.x,
                                                                   m_internals->threedFiducials[m].positionMM.y,
                                                                   m_internals->threedFiducials[m].positionMM.z);
    }

    // maybe this helps with error 13?
    Sleep(50.0 * cmn_ms);
}

void mtsAtracsysFusionTrack::Cleanup(void)
{
    ftkClose(&m_internals->Library);
}


bool mtsAtracsysFusionTrack::AddTool(const std::string & toolName,
                                     const std::string & fileName,
                                     const bool isJson)
{

    // check if this tool already exists
    auto tool_iterator = m_tools.find(toolName);
    if (tool_iterator != m_tools.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "AddTool: " << toolName << " already exists" << std::endl;
        return false;
    }

    // make sure we can find and load this tool ini file
    ftkGeometry geometry;
    bool parse;
    if (isJson) {
        parse = LoadGeometryJSON(fileName, geometry);
    } else {
        parse = LoadGeometryINI(fileName, geometry);
    }

    if (parse) {
        CMN_LOG_CLASS_INIT_VERBOSE << "AddTool: loaded geometry from file \""
                                   << fileName << "\" for tool \"" << toolName << "\"" << std::endl;
        for (size_t index = 0; index < geometry.pointsCount; ++index) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Marker[" << index << "] = ("
                                       << geometry.positions[index].x << ", "
                                       << geometry.positions[index].y << ", "
                                       << geometry.positions[index].z << ")" << std::endl;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddTool: failed to parse geometry file \""
                                 << fileName << "\" for tool \"" << toolName << "\"" << std::endl;
        return false;
    }

    ftkError error = ftkSetGeometry(m_internals->Library, m_internals->Device, &geometry);
    if (error != FTK_OK) {
        CMN_LOG_CLASS_INIT_ERROR << "AddTool: unable to set geometry for tool \"" << toolName
                                 << "\" using geometry file \"" << fileName
                                 << "\" (" << this->GetName() << ")" << std::endl;
        return false;
    }
    else {
        CMN_LOG_CLASS_INIT_VERBOSE << "AddTool: set geometry for tool \"" << toolName
                                   << "\" using geometry file \"" << fileName
                                   << "\" successful (" << this->GetName() << ")"
                                   << std::endl;
    }

    // make sure there is no such geometry Id yet
    const mtsAtracsysFusionTrackInternals::GeometryIdToToolMap::const_iterator
        toolIterator = m_internals->GeometryIdToTool.find(geometry.geometryId);
    if (toolIterator != m_internals->GeometryIdToTool.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "AddTool: error, found an existing tool with the same Id "
                                 << geometry.geometryId << " for tool \"" << toolName << "\"" << std::endl;
        return false;
    }

    // finally create a cisst tool structure
    auto tool = new mtsAtracsysFusionTrackTool(toolName);

    // create an interface for tool
    tool->m_interface = AddInterfaceProvided(toolName);
    if (!tool->m_interface) {
        CMN_LOG_CLASS_INIT_ERROR << "AddTool: " << tool->m_name << " already exists" << std::endl;
        delete tool;
        return false;
    }

    // register newly created tool
    this->m_tools[toolName] = tool;
    m_internals->GeometryIdToTool[geometry.geometryId] = tool;

    // frames used
    tool->m_measured_cp.ReferenceFrame() = this->Name;
    tool->m_measured_cp.MovingFrame() = toolName;

    // add data for this tool and populate tool interface
    tool->m_state_table.SetAutomaticAdvance(false);
    this->AddStateTable(&(tool->m_state_table));
    tool->m_state_table.AddData(tool->m_measured_cp, "measured_cp");
    tool->m_state_table.AddData(tool->m_registration_error, "registration_error");
    tool->m_interface->AddCommandReadState(tool->m_state_table, tool->m_measured_cp, "measured_cp");
    tool->m_interface->AddCommandReadState(tool->m_state_table, tool->m_registration_error, "registration_error");
    tool->m_interface->AddCommandReadState(tool->m_state_table,
                                           tool->m_state_table.PeriodStats,
                                           "period_statistics");

    // update list of crtk interfaces
    m_configuration_state_table.Start();
    m_crtk_interfaces_provided.push_back(mtsDescriptionInterfaceFullName("localhost", this->Name, toolName));
    m_configuration_state_table.Advance();
    m_crtk_interfaces_provided_updated();

    return true;
}

std::string mtsAtracsysFusionTrack::GetToolName(const size_t index) const
{
    ToolsType::const_iterator toolIterator = m_tools.begin();
    if (index >= m_tools.size()) {
        CMN_LOG_CLASS_RUN_ERROR << "GetToolName: requested index is out of range" << std::endl;
        return "";
    }
    for (unsigned int i = 0; i < index; i++) {
        toolIterator++;
    }
    return toolIterator->first;
}
