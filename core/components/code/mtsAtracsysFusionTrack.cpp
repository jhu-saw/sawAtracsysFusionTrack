/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2021 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackConfig.h>
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
    mtsAtracsysFusionTrackInternals():
        m_library(0),
        m_sn(0),
        m_frame_query(0)
    {}

    void SetupFrameQuery(const size_t number_of_tools,
                         const size_t number_of_stray_markers) {
        if (m_frame_query) {
            delete m_frame_query;
        }
        m_frame_query = ftkCreateFrame();
        // memset(m_frame_query, 0, sizeof(ftkFrameQuery));  // lead to error 4 with SDK 4.4.1

        // tools, aka fT Markers
        m_tools = new ftkMarker[number_of_tools];
        m_frame_query->markers = m_tools;
        m_frame_query->markersVersionSize.ReservedSize = sizeof(ftkMarker) * number_of_tools;
        // stray markers, aka fT 3D fiducials
        m_stray_markers = new ftk3DFiducial[number_of_stray_markers];
        m_frame_query->threeDFiducials = m_stray_markers;
        m_frame_query->threeDFiducialsVersionSize.ReservedSize = sizeof(ftk3DFiducial) * number_of_stray_markers;

        ftkError err(ftkSetFrameOptions(false, false, 16u, 16u,
                                        // 0u, 16u,
                                        number_of_stray_markers, number_of_tools,
                                        m_frame_query));
        if (err != FTK_ERROR_NS::FTK_OK) {
            CMN_LOG_INIT_ERROR << "mtsAtracsysFusionTrackInternals: ftkSetFrameOptions failed" << std::endl;
        } else {
            CMN_LOG_INIT_VERBOSE << "mtsAtracsysFusionTrackInternals: ftkSetFrameOptions ok" << std::endl;
        }
    }

    ftkLibrary m_library;
    uint64 m_sn;
    ftkFrameQuery * m_frame_query;
    ftkMarker * m_tools;
    ftk3DFiducial * m_stray_markers;
    bool m_configured;

    typedef std::map<uint32, mtsAtracsysFusionTrackTool *> GeometryIdToToolMap;
    GeometryIdToToolMap GeometryIdToTool;
};


static void mtsAtracsysFusionTrackDeviceEnum(uint64 device, void * user, ftkDeviceType CMN_UNUSED(type))
{
    uint64 * lastDevice = reinterpret_cast<uint64 *>(user);
    if (lastDevice) {
        *lastDevice = device;
    }
}


void mtsAtracsysFusionTrack::Init(void)
{
    m_stray_markers_max = 10;
    m_internals = new mtsAtracsysFusionTrackInternals();

    // configuration state table
    m_configuration_state_table.SetAutomaticAdvance(false);
    AddStateTable(&m_configuration_state_table);
    m_configuration_state_table.AddData(m_crtk_interfaces_provided, "crtk_interfaces_provided");

    StateTable.AddData(m_measured_cp_array, "measured_cp_array");

    m_controller_interface = AddInterfaceProvided("Controller");
    if (m_controller_interface) {
        // system info
        m_controller_interface->AddMessageEvents();
        m_controller_interface->AddCommandReadState(StateTable, m_measured_cp_array, "measured_cp_array");
        m_controller_interface->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");

        // crtk interfaces
        m_controller_interface->AddCommandReadState(m_configuration_state_table, m_crtk_interfaces_provided,
                                                    "crtk_interfaces_provided");
        m_controller_interface->AddEventVoid(m_crtk_interfaces_provided_updated, "crtk_interfaces_provided_updated");
    }
}

#if 0
static void OptionEnumerator(uint64 sn, void *CMN_UNUSED(user), ftkOptionsInfo *oi)
{
    CMN_LOG_INIT_VERBOSE << "Option " << oi->id << "  " << oi->name << std::endl;
}
#endif

void mtsAtracsysFusionTrack::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;
    ftkBuffer buffer;

    ftkVersion(&buffer);
    CMN_LOG_CLASS_INIT_VERBOSE << "Atracsys SDK Version " << buffer.data << std::endl;

#if 0
    if (ftkEnumerateOptions(m_internals->m_library, 0LL, OptionEnumerator, NULL) != SDK_FTK_OK)
        CMN_LOG_CLASS_INIT_WARNING << "Configure: failed to enumerate Atracsys options" << std::endl;

    if (ftkGetData(m_internals->m_library, 0LL, FTK_OPT_DRIVER_VER, &buffer) != SDK_FTK_OK)
        CMN_LOG_CLASS_INIT_WARNING << "Configure: failed to get Atracsys Driver version" << std::endl;
#endif

    // initialize fusion track library
    m_internals->m_library = ftkInit();
    if (!m_internals->m_library) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to initialize ("
                                 << this->GetName() << ")" << std::endl;
        return;
    }

    for (size_t i = 0; i < 10; i++) {
        // search for devices
        ftkError error = ftkEnumerateDevices(m_internals->m_library,
                                             mtsAtracsysFusionTrackDeviceEnum,
                                             &(m_internals->m_sn));
        if (error != FTK_ERROR_NS::FTK_OK) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to enumerate devices ("
                                     << this->GetName() << ")" << std::endl;
            ftkClose(&m_internals->m_library);
        }

        if (m_internals->m_sn != 0LL) {
            break;
        }
    }
    if (m_internals->m_sn == 0LL) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: no device connected ("
                                 << this->GetName() << ")" << std::endl;
        ftkClose(&m_internals->m_library);
        return;
    }

    if (filename == "") {
        m_internals->SetupFrameQuery(m_tools.size(), m_stray_markers_max);
        return;
    }

    // allows the use of relative paths for geometry files
    m_path.Add(cmnPath::GetWorkingDirectory());
    m_path.Add(std::string(sawAtracsysFusionTrack_SOURCE_DIR) + "/../share", cmnPath::TAIL);

    std::string fullname = m_path.Find(filename);
    if (!cmnPath::Exists(fullname)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: configuration file \"" << filename
                                 << "\" not found in path (" << m_path << ")" << std::endl;
        exit(EXIT_FAILURE);
    }

    // added path of main config file so we can load relative geometry files
    std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
    m_path.Add(configDir, cmnPath::TAIL);

    // read JSON file passed as param, see configAtracsysFusionTrack.json for an example
    std::ifstream jsonStream;
    jsonStream.open(fullname.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        m_internals->m_configured = false;
        return;
    }

#ifdef FTK_OPT_DATA_DIR
    // FTK_OPT_DATA_DIR is defined in SDK 3.0.1, but not in SDK 4.5.2
    // add FTK path too
    if ((ftkGetData(m_internals->m_library, m_internals->m_sn,
                    FTK_OPT_DATA_DIR, &buffer ) == FTK_ERROR_NS::FTK_OK) && (buffer.size > 0)) {
        std::string ftkPath(reinterpret_cast<char*>(buffer.data));
        configPath.Add(ftkPath);
    }
#endif

    // path to locate tool definitions
    const Json::Value definitionPath = jsonConfig["definition-path"];
    // preserve order from config file
    for (int index = (definitionPath.size() - 1);
         index >= 0;
         --index) {
        std::string path = definitionPath[index].asString();
        if (path != "") {
            m_path.Add(path, cmnPath::HEAD);
        }
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
                exit(EXIT_FAILURE);
            } else if (!iniTool.empty()) {
                isJson = false;
                toolFile = iniTool.asString();
            } else {
                isJson = true;
                toolFile = jsonTool.asString();
            }

            fullname = m_path.Find(toolFile);
            // make sure ini file is valid
            if (cmnPath::Exists(fullname)) {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: calling AddTool with tool name: " << toolName << " and configuration file: " << filename << std::endl;
                AddTool(toolName, fullname, isJson);
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: configuration file \"" << toolFile
                                         << "\" for tool \"" << toolName
                                         << "\" not found in path (" << m_path << ")" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }

    // finally, prepare frame query
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: calling SetupFrameQuery for " << m_tools.size() << " tool(s) and up to "
                               << m_stray_markers_max << " stray marker(s)" << std::endl;
    m_internals->SetupFrameQuery(m_tools.size(), m_stray_markers_max);
    m_internals->m_configured = true;
}


void mtsAtracsysFusionTrack::Startup(void)
{
    CMN_LOG_CLASS_RUN_ERROR << "Startup" << std::endl;
    // trigger event so connected component can bridge crtk provided interface as needed
    m_configuration_state_table.Start();
    m_crtk_interfaces_provided.push_back(mtsDescriptionInterfaceFullName("localhost", this->Name, "Controller"));
    m_configuration_state_table.Advance();
    m_crtk_interfaces_provided_updated();

    // reports system found
    m_controller_interface->SendStatus(this->GetName() + ": found device SN " + std::to_string(m_internals->m_sn));
    // set reference frame for measured_cp_array
    m_measured_cp_array.ReferenceFrame() = this->Name;
}


void mtsAtracsysFusionTrack::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    // get latest frame from fusion track library/device
    ftkError status = ftkGetLastFrame(m_internals->m_library,
                                      m_internals->m_sn,
                                      m_internals->m_frame_query,
                                      100u);
    // negative error codes are warnings
    m_measured_cp_array.SetValid(true);
    if (status != FTK_ERROR_NS::FTK_OK) {
        if (static_cast<int>(status) < 0) {
            // std::cerr << "Warning: " << status << std::endl;
        } else {
            m_measured_cp_array.SetValid(false);
            std::cerr << "Error: " << static_cast<int>(status) << std::endl;
            return;
        }
    }

    // check results of last frame
    switch (m_internals->m_frame_query->markersStat) {
    case FTK_QS_NS::QS_WAR_SKIPPED:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: marker fields in the frame are not set correctly" << std::endl;
        break;
    case FTK_QS_NS::QS_ERR_INVALID_RESERVED_SIZE:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: frame.markersVersionSize is invalid" << std::endl;
        break;
    default:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
        break;
    case FTK_QS_NS::QS_OK:
        break;
    }

    // make sure we're not getting more markers than allocated
    size_t count = m_internals->m_frame_query->markersCount;
    if (count > m_tools.size()) {
        CMN_LOG_CLASS_RUN_WARNING << "Run: marker overflow, please increase number of markers.  Only the first "
                                  << m_tools.size() << " marker(s) will processed." << std::endl;
        count = m_tools.size();
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
        ftkMarker & ft_tool = m_internals->m_tools[index];

        // find the appropriate tool
        uint32 id = ft_tool.geometryId;
        if (m_internals->GeometryIdToTool.find(id) == m_internals->GeometryIdToTool.end()) {
            CMN_LOG_CLASS_RUN_WARNING << "Run: found a geometry Id ("
                                      << id << ") not registered using AddTool, this marker will be ignored ("
                                      << this->GetName() << ")" << std::endl;
        }
        else {
            mtsAtracsysFusionTrackTool * tool = m_internals->GeometryIdToTool.at(id);
            tool->m_measured_cp.SetValid(true);
            tool->m_measured_cp.Position().Translation().Assign(ft_tool.translationMM[0] * cmn_mm,
                                                                ft_tool.translationMM[1] * cmn_mm,
                                                                ft_tool.translationMM[2] * cmn_mm);
            for (size_t row = 0; row < 3; ++row) {
                for (size_t col = 0; col < 3; ++col) {
                    tool->m_measured_cp.Position().Rotation().Element(row, col) = ft_tool.rotation[row][col];
                }
            }
            tool->m_registration_error = ft_tool.registrationErrorMM * cmn_mm;
        }
    }

    // finalize all tools
    for (iter = m_tools.begin(); iter != end; ++iter) {
        iter->second->m_state_table.Advance();
    }

    // ---- 3D Fiducials, aka stray markers ---
    switch (m_internals->m_frame_query->threeDFiducialsStat) {
    case FTK_QS_NS::QS_WAR_SKIPPED:
        CMN_LOG_CLASS_RUN_ERROR << "Run: 3D status fields in the frame is not set correctly" << std::endl;
        break;
    case FTK_QS_NS::QS_ERR_INVALID_RESERVED_SIZE:
        CMN_LOG_CLASS_RUN_ERROR << "Run: frame.threeDFiducialsVersionSize is invalid" << std::endl;
        break;
    default:
        // CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl; 
        break;
    case FTK_QS_NS::QS_OK:
        break;
    }

    size_t stray_count = m_internals->m_frame_query->threeDFiducialsCount;
    m_measured_cp_array.Positions().resize(stray_count);

    //printf("3D fiducials:\n");
    for (uint32 m = 0; m < stray_count; m++) {
#if 0
          printf("\tINDEXES (%u %u)\t XYZ (%.2f %.2f %.2f)\n\t\tEPI_ERR: %.2f\tTRI_ERR: %.2f\tPROB: %.2f\n",
          m_internals->m_stray_markers[m].leftIndex,
          m_internals->m_stray_markers[m].rightIndex,
          m_internals->m_stray_markers[m].positionMM.x,
          m_internals->m_stray_markers[m].positionMM.y,
          m_internals->m_stray_markers[m].positionMM.z,
          m_internals->m_stray_markers[m].epipolarErrorPixels,
          m_internals->m_stray_markers[m].triangulationErrorMM,
          m_internals->m_stray_markers[m].probability);
#endif
        m_measured_cp_array.Positions().at(m).Translation().Assign(m_internals->m_stray_markers[m].positionMM.x * cmn_mm,
                                                                   m_internals->m_stray_markers[m].positionMM.y * cmn_mm,
                                                                   m_internals->m_stray_markers[m].positionMM.z * cmn_mm);
    }
    // maybe this helps with error 13?
    // Sleep(50.0 * cmn_ms);  // no needed with 4.4.1?
}

void mtsAtracsysFusionTrack::Cleanup(void)
{
    ftkClose(&m_internals->m_library);
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
        exit(EXIT_FAILURE);
    }

    ftkError error = ftkSetGeometry(m_internals->m_library, m_internals->m_sn, &geometry);
    if (error != FTK_ERROR_NS::FTK_OK) {
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
    CMN_LOG_CLASS_INIT_VERBOSE << "AddTool: registering tool " << toolName
                               << " with geometry Id " << geometry.geometryId << std::endl;
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
