/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawAtracsysFusionTrack/mtsAtracsysFusionTrack.h>

#include <ftkInterface.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackConfig.h>
#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackRevision.h>


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
    prmPositionCartesianGet m_local_measured_cp;
    prmPositionCartesianGet m_measured_cp;
    prmPositionCartesianGet * m_reference_measured_cp = nullptr;
    double m_registration_error;
    mtsStateTable m_state_table;
};

class mtsAtracsysFusionTrackInternals
{
public:
    mtsAtracsysFusionTrackInternals():
        m_library(nullptr),
        m_sn(0),
        m_frame_query(nullptr),
        m_configured(false),
        m_images_enabled(false)
    {}

    ~mtsAtracsysFusionTrackInternals() {
        if (m_frame_query != nullptr) {
            delete m_frame_query;
        }

        if (m_library != nullptr) {
            ftkClose(&m_library);
        }
    }

    bool Initialize()
    {
        ftkBuffer buffer;
        ftkVersion(&buffer);
        CMN_LOG_INIT_VERBOSE << "Atracsys SDK Version " << buffer.data << std::endl;

        // initialize SDK
        m_library = ftkInit();
        if (m_library == nullptr) {
            CMN_LOG_INIT_ERROR << "Configure: unable to initialize Atracsys SDK" << std::endl;
            return false;
        }

        size_t max_attempts = 10;
        for (size_t attempt = 0; attempt < max_attempts; attempt++) {
            // search for devices
            ftkError error = ftkEnumerateDevices(m_library, DeviceEnumerateCallback, this);
            if (error != FTK_ERROR_NS::FTK_OK) {
                CMN_LOG_INIT_ERROR << "Configure: unable to enumerate Atracsys devices" << std::endl;
            }

            if (m_sn != 0) {
                break;
            }
        }

        if (m_sn == 0) {
            CMN_LOG_INIT_ERROR << "Configure: no Atracsys devices found" << std::endl;
            return false;
        }

        ftkError error = ftkEnumerateOptions(m_library, 0, OptionEnumerateCallback, this);
        if (error != FTK_ERROR_NS::FTK_OK && error != ftkError::FTK_WAR_OPT_GLOBAL_ONLY) {
            CMN_LOG_INIT_ERROR << "Configure: unable to enumerate supported global library options" << std::endl;
        }

        error = ftkEnumerateOptions(m_library, m_sn, OptionEnumerateCallback, this);
        if (error != FTK_ERROR_NS::FTK_OK) {
            CMN_LOG_INIT_ERROR << "Configure: unable to enumerate supported device options" << std::endl;
        }

        return true;
    }

    void SetupFrameQuery(const size_t number_of_tools,
                         const size_t number_of_stray_markers)
    {
        if (m_frame_query) {
            delete m_frame_query;
        }
        m_frame_query = ftkCreateFrame();

        // tools, aka fT Markers
        m_tools = new ftkMarker[number_of_tools];
        m_frame_query->markers = m_tools;
        m_frame_query->markersVersionSize.ReservedSize = sizeof(ftkMarker) * number_of_tools;
        // stray markers, aka fT 3D fiducials
        m_stray_markers = new ftk3DFiducial[number_of_stray_markers];
        m_frame_query->threeDFiducials = m_stray_markers;
        m_frame_query->threeDFiducialsVersionSize.ReservedSize = sizeof(ftk3DFiducial) * number_of_stray_markers;

        ftkError err(ftkSetFrameOptions(m_images_enabled, false, 16u, 16u,
                                        number_of_stray_markers, number_of_tools,
                                        m_frame_query));
        if (err != FTK_ERROR_NS::FTK_OK) {
            CMN_LOG_INIT_ERROR << "mtsAtracsysFusionTrackInternals: ftkSetFrameOptions failed" << std::endl;
        } else {
            CMN_LOG_INIT_VERBOSE << "mtsAtracsysFusionTrackInternals: ftkSetFrameOptions ok" << std::endl;
            m_configured = true;
        }
    }

    std::string QueryDeviceType()
    {
        auto option = m_device_options.find(option_sTk_device_type);
        if (option == m_device_options.end()) { return ""; }

        ftkBuffer buffer;
        ftkError status = ftkGetData(m_library, m_sn, option->second.id, &buffer);
        if (status != ftkError::FTK_OK) return "";

        std::string device_type(buffer.data);

        return device_type;
    }

    bool ConfigureImageSending(bool enabled, int sl_period, int dots)
    {
        bool ok;

        // No structured light, IR frames only
        if (sl_period <= 0) {
            ok = SetStringOption(option_image_pattern, "I");
        // One structured light frame per period, others are IR
        } else {
            std::string pattern = std::string(sl_period - 1, 'I') + "S";
            ok = SetStringOption(option_image_pattern, pattern);
        }

        if (!ok) { return false; }

        int32_t enabled_value = enabled ? 1 : 0;
        ok = SetIntOption(option_enable_images_sending, enabled_value);
        if (ok) {
            m_images_enabled = enabled;
        } else {
            return false;
        }

        ok = SetIntOption(option_sl_integration_time, 16000);
        if (!ok) { return false; }

        ok = SetIntOption(option_num_enabled_dot_projector, dots);
        if (!ok) { return false; }

        ok = SetIntOption(option_dot_projector_strobe_time, 8000);
        if (!ok) { return false; }

        return true;
    }

    void ExtractImage(uint8_t *pixels, prmImageFrame& dest)
    {
        dest.Width() = m_frame_query->imageHeader->width;
        dest.Height() = m_frame_query->imageHeader->height;
        dest.Channels() = 1;

        const size_t size = dest.Width() * dest.Height();
        dest.Data().resize(size);
        std::copy(pixels, pixels + size, dest.Data().begin());
    }

    void ExtractLeftImage(prmImageFrame& dest)
    {
        ExtractImage(m_frame_query->imageLeftPixels, dest);
    }

    void ExtractRightImage(prmImageFrame& dest)
    {
        ExtractImage(m_frame_query->imageRightPixels, dest);
    }

#if AtracsysSDK_HAS_ftkCameraParameters
    // Convert ftkCameraParameters to standard 3x3 intrinsic matrix representation
    void CameraParamsToIntrinsicMatrix(ftkCameraParameters& params, vct3x3& intrinsic_out)
    {
        double fu = params.FocalLength[0];
        double fv = params.FocalLength[1];

        intrinsic_out.at(0, 0) = fu;
        intrinsic_out.at(1, 1) = fv;
        intrinsic_out.at(0, 1) = fu * params.Skew;
        intrinsic_out.at(0, 2) = params.OpticalCentre[0];
        intrinsic_out.at(1, 2) = params.OpticalCentre[1];
        intrinsic_out.at(2, 2) = 1.0f;
    }

    // Retrieve stereo camera calibration from device, return true/false for success/error
    bool RetrieveStereoCameraCalibration(prmCameraInfo& left, prmCameraInfo& right, vct3& rotation, vct3& translation)
    {
        bool ok = SetIntOption(option_export_calibration, 1);
        if (!ok) { return false; }

        // need initialized frame query to retrieve camera calibration
        if (!m_frame_query) {
            SetupFrameQuery(0, 0);
        }

        int attempts = 0;
        int max_attempts = 20;
        ftkError status;

        do {
            status = ftkGetLastFrame(m_library, m_sn, m_frame_query, 100u);
        } while (status != ftkError::FTK_OK && attempts++ < max_attempts);

        if (status != ftkError::FTK_OK) {
            CMN_LOG_RUN_ERROR << "Error retrieving stereo calibration frame: "
                              << static_cast<int>(status) << std::endl;
            return false;
        }

        ok = SetIntOption(option_export_calibration, 0);
        if (!ok) { return false; }

        ftkFrameInfoData frame_info;
        frame_info.WantedInformation = ftkInformationType::CalibrationParameters;

        status = ftkExtractFrameInfo(m_frame_query, &frame_info);
        if (status != ftkError::FTK_OK) {
            CMN_LOG_RUN_ERROR << "Error extracting frame info for stereo calibration: "
                              << static_cast<int>(status) << std::endl;
            return false;
        }

        ftkStereoParameters stereo_params = frame_info.Calibration;
        ftkCameraParameters left_params = stereo_params.LeftCamera;
        ftkCameraParameters right_params = stereo_params.RightCamera;

        auto header = m_frame_query->imageHeader;
        left.Width() = right.Width() = header->width;
        left.Height() = right.Height() = header->height;

        left.DistortionParameters().SetSize(5);
        right.DistortionParameters().SetSize(5);
        for (size_t idx = 0; idx < 5; idx++) {
            left.DistortionParameters()[idx] = left_params.Distorsions[idx];
            right.DistortionParameters()[idx] = right_params.Distorsions[idx];
        }

        CameraParamsToIntrinsicMatrix(left_params, left.Intrinsic());
        CameraParamsToIntrinsicMatrix(right_params, right.Intrinsic());

        for (size_t idx = 0; idx < 3; idx++) {
            rotation[idx] = stereo_params.Rotation[idx];
            translation[idx] = stereo_params.Translation[idx] * cmn_mm;
        }

        return true;
    }
#endif

    ftkLibrary m_library;
    uint64 m_sn;
    ftkDeviceType m_device_type;
    ftkFrameQuery * m_frame_query;
    ftkMarker * m_tools;
    ftk3DFiducial * m_stray_markers;
    bool m_configured;

    bool m_images_enabled;

    typedef std::map<uint32, mtsAtracsysFusionTrackTool *> GeometryIdToToolMap;
    GeometryIdToToolMap GeometryIdToTool;

private:
    std::map<std::string, ftkOptionsInfo> m_device_options;

    const std::string option_sTk_device_type = "sTk device type";
    const std::string option_enable_images_sending = "Enable images sending";
    const std::string option_image_pattern = "Image Scheduler Pattern";;
    const std::string option_vis_integration_time = "Image Integration Time for VIS frames";
    const std::string option_sl_integration_time = "Image Integration Time for SL frames";
    const std::string option_num_enabled_dot_projector = "Enable dot projectors";
    const std::string option_dot_projector_strobe_time = "Dot projectors Strobe Time for SL frames";
    const std::string option_export_calibration = "Calibration export";

    bool SetIntOption(const std::string& option_name, int32_t value) {
        auto option = m_device_options.find(option_name);
        if (option == m_device_options.end()) {
            CMN_LOG_RUN_ERROR << "option \"" << option_name << "\" not supported by device" << std::endl;
            return false;
        }

        ftkError status;
        auto id = option->second.id;
        int32_t min, max;
        status = ftkGetInt32(m_library, m_sn, id, &min, ftkOptionGetter::FTK_MIN_VAL);
        if (status != ftkError::FTK_OK) {
            CMN_LOG_INIT_ERROR << "error getting min for device option \"" << option_name << "\""
                               << ": " << static_cast<int>(status) << std::endl;
            return false;
        }

        status = ftkGetInt32(m_library, m_sn, id, &max, ftkOptionGetter::FTK_MAX_VAL);
        if (status != ftkError::FTK_OK) {
            CMN_LOG_INIT_ERROR << "error getting max for device option \"" << option_name << "\""
                               << ": " << static_cast<int>(status) << std::endl;
            return false;
        }

        if (value < min || value > max) {
            CMN_LOG_RUN_ERROR << "value " << value
                              << " out of range [" << min << ".." << max << "]"
                              << " for device option \"" << option_name << "\"" << std::endl;
            return false;
        }

        status = ftkSetInt32(m_library, m_sn, id, value);
        if (status != ftkError::FTK_OK) {
            CMN_LOG_INIT_ERROR << "error setting device option \"" << option_name << "\""
                               << ": " << static_cast<int>(status) << std::endl;
            return false;
        }

        return true;
    }

    bool SetStringOption(const std::string& option_name, std::string value) {
        auto option = m_device_options.find(option_name);
        if (option == m_device_options.end()) {
            CMN_LOG_RUN_ERROR << "option \"" << option_name << "\" not supported by device" << std::endl;
            return false;
        }

        auto id = option->second.id;

        ftkBuffer buffer;
        buffer.reset();
        std::memcpy(buffer.uData, value.c_str(), value.length());
        buffer.size = value.length();

        ftkError status = ftkSetData(m_library, m_sn, id, &buffer);
        if (status != ftkError::FTK_OK) {
            CMN_LOG_INIT_ERROR << "error setting device option \"" << option_name << "\""
                               << ": " << static_cast<int>(status) << std::endl;
            return false;
        }

        return true;
    }

    static void DeviceEnumerateCallback(uint64 device, void* user, ftkDeviceType type)
    {
        mtsAtracsysFusionTrackInternals* internals = reinterpret_cast<mtsAtracsysFusionTrackInternals*>(user);
        if (!internals) { return; }

        internals->m_sn = device;
        internals->m_device_type = type;
    }

    static void OptionEnumerateCallback(uint64 CMN_UNUSED(sn), void* user, ftkOptionsInfo* oi)
    {
        mtsAtracsysFusionTrackInternals* internals = reinterpret_cast<mtsAtracsysFusionTrackInternals*>(user);
        if (!internals) { return; }

        internals->m_device_options[oi->name] = *oi;
    }
};

void mtsAtracsysFusionTrack::Init(void)
{
    m_stray_markers_max = 10;
    m_internals = new mtsAtracsysFusionTrackInternals();

    // configuration state table
    m_configuration_state_table.SetAutomaticAdvance(false);
    AddStateTable(&m_configuration_state_table);
    m_configuration_state_table.AddData(m_crtk_interfaces_provided, "crtk_interfaces_provided");

    m_configuration_state_table.AddData(m_left_camera_info, "left_camera_info");
    m_configuration_state_table.AddData(m_right_camera_info, "right_camera_info");
    m_configuration_state_table.AddData(m_camera_rotation, "camera_rotation");
    m_configuration_state_table.AddData(m_camera_translation, "camera_translation");

    StateTable.AddData(m_measured_cp_array, "measured_cp_array");
    StateTable.AddData(m_left_image_raw, "left_image");
    StateTable.AddData(m_right_image_raw, "right_image");

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

    m_stereo_raw_interface = AddInterfaceProvided("StereoRaw");
    if (m_stereo_raw_interface) {
        m_stereo_raw_interface->AddCommandReadState(StateTable, m_left_image_raw, "left/image_raw");
        m_stereo_raw_interface->AddCommandReadState(StateTable, m_right_image_raw, "right/image_raw");
        m_stereo_raw_interface->AddCommandReadState(m_configuration_state_table, m_left_camera_info, "left/camera_info");
        m_stereo_raw_interface->AddCommandReadState(m_configuration_state_table, m_right_camera_info, "right/camera_info");
        m_stereo_raw_interface->AddCommandReadState(m_configuration_state_table, m_camera_rotation, "camera_rotation");
        m_stereo_raw_interface->AddCommandReadState(m_configuration_state_table, m_camera_translation, "camera_translation");
    }
}

void mtsAtracsysFusionTrack::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    bool ok = m_internals->Initialize();
    if (!ok) {
        CMN_LOG_INIT_ERROR << "Configure: failed to initialize device ("
                                 << this->GetName() << ")" << std::endl;
        return;
    }

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found device SN " << m_internals->m_sn << std::endl;

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
        exit(EXIT_FAILURE);
        return;
    }

#ifdef FTK_OPT_DATA_DIR
    // FTK_OPT_DATA_DIR is defined in SDK 3.0.1, but not in SDK 4.5.2
    // add FTK path too
    if ((ftkGetData(m_internals->m_library, m_internals->m_sn,
                    FTK_OPT_DATA_DIR, &buffer ) == FTK_ERROR_NS::FTK_OK) && (buffer.size > 0)) {
        std::string ftkPath(reinterpret_cast<char*>(buffer.data));
        m_path.Add(ftkPath);
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
        if ((toolName == "") || (toolName == this->Name)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid tool name found in " << filename
                                     << ".  The tool name must be a non empty string and must not match the device's name ("
                                     << this->Name << ")" << std::endl;
        } else {

            // reference frame
            std::string reference;
            const Json::Value jsonReference = jsonValue["reference"];
            if (!jsonValue.empty()) {
                reference = jsonReference.asString();
            }

            const Json::Value iniTool = jsonValue["ini-file"];
            const Json::Value jsonTool = jsonValue["json-file"];
            if (iniTool.empty() && jsonTool.empty()) {
                CMN_LOG_INIT_ERROR << "Configure: you need to define either \"ini-file\" or \"json-file\" for the tool \""
                                         << toolName << "\" in configuration file \""
                                         << filename << "\", neither was found" << std::endl;
            } else if (!iniTool.empty() && !jsonTool.empty()) {
                CMN_LOG_INIT_ERROR << "Configure: you need to define either \"ini-file\" or \"json-file\" for the tool \""
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
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: calling AddTool with tool name: "
                                           << toolName << " and configuration file: " << filename
                                           << ", reference: " << reference << std::endl;
                if (!AddTool(toolName, fullname, isJson, reference)) {
                    exit(EXIT_FAILURE); 
                }
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: configuration file \"" << toolFile
                                         << "\" for tool \"" << toolName
                                         << "\" not found in path (" << m_path << ")" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }

    // if stereo is configured, configure Atracsys to send images
    bool has_stereo_config = jsonConfig.isMember("stereo");
    if (has_stereo_config) {
        std::string device_type = m_internals->QueryDeviceType();
        if (device_type == "") {
            CMN_LOG_CLASS_INIT_ERROR << "failed to read device type" << std::endl;
        } else {
            CMN_LOG_CLASS_INIT_VERBOSE << "device type: " << device_type << std::endl;
        }

        const Json::Value stereo_config = jsonConfig["stereo"];

        const Json::Value video = stereo_config["video"];
        bool video_enabled = stereo_config.isMember("video") && video.isBool() && video.asBool();
        const Json::Value depth = stereo_config["depth"];
        bool depth_enabled = stereo_config.isMember("depth") && depth.isBool() && depth.asBool();
        // video processing is required for depth map computation
        video_enabled = depth_enabled || video_enabled;

        const Json::Value dots_config = stereo_config["num_dot_projectors"];
        bool dots_configured = stereo_config.isMember("num_dot_projectors") && dots_config.isInt();
        int enabled_dot_projectors = dots_configured ? dots_config.asInt() : 1;
        if (!depth_enabled) {
            enabled_dot_projectors = 0;
        }

        int sl_period = depth_enabled ? 5 : 0;

        bool ok = m_internals->ConfigureImageSending(video_enabled, sl_period, enabled_dot_projectors);
        if (!ok) {
            CMN_LOG_CLASS_INIT_ERROR << "failed to enabled image sending" << std::endl;
        }
    }

    // finally, prepare frame query
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: calling SetupFrameQuery for " << m_tools.size() << " tool(s) and up to "
                               << m_stray_markers_max << " stray marker(s)" << std::endl;
    m_internals->SetupFrameQuery(m_tools.size(), m_stray_markers_max);
}


void mtsAtracsysFusionTrack::Startup(void)
{
    m_configuration_state_table.Start();

    // reports system found
    std::string status_message = this->GetName() + ": found device SN " + std::to_string(m_internals->m_sn)
                                         + ", type " + m_internals->QueryDeviceType();
    m_controller_interface->SendStatus(status_message);
    // set reference frame for measured_cp_array
    m_measured_cp_array.ReferenceFrame() = this->Name;

#if AtracsysSDK_HAS_ftkCameraParameters
    bool ok = m_internals->RetrieveStereoCameraCalibration(
        m_left_camera_info, m_right_camera_info,
        m_camera_rotation, m_camera_translation
    );
#else
    bool ok = false;
#endif
    m_left_camera_info.Valid() = ok;
    m_right_camera_info.Valid() = ok;

    m_configuration_state_table.Advance();

    m_crtk_interfaces_provided_updated();
}


void mtsAtracsysFusionTrack::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    if (!HardwareInitialized()) {
        return;
    }

    // get latest frame from fusion track library/device
    ftkError status = ftkGetLastFrame(m_internals->m_library,
                                      m_internals->m_sn,
                                      m_internals->m_frame_query,
                                      100u);

    // negative error codes are warnings
    if (status != FTK_ERROR_NS::FTK_OK) {
        if (static_cast<int>(status) < 0) {
            CMN_LOG_CLASS_RUN_WARNING << "Warning: " << static_cast<int>(status) << std::endl;
        } else {
            m_measured_cp_array.SetValid(false);
            CMN_LOG_CLASS_RUN_ERROR << "Error: " << static_cast<int>(status) << std::endl;
            return;
        }
    }

    m_measured_cp_array.SetValid(true);

    ftkPixelFormat frame_format = m_internals->m_frame_query->imageHeader->format;
    switch (frame_format)
    {
        case ftkPixelFormat::GRAY8:
        case ftkPixelFormat::GRAY16:
            ProcessIRTrackingFrame();
            break;
#if AtracsysSDK_HAS_ftkCameraParameters
        case ftkPixelFormat::GRAY8_VIS:
        case ftkPixelFormat::GRAY16_VIS:
        case ftkPixelFormat::GRAY8_SL:
        case ftkPixelFormat::GRAY16_SL:
            ProcessRGBStereoFrame();
            break;
#endif
        default:
            CMN_LOG_CLASS_RUN_WARNING << "Warning: unknown frame format: " << static_cast<int>(frame_format) << std::endl;
            break;
    }
}


void mtsAtracsysFusionTrack::Cleanup(void) {}


bool mtsAtracsysFusionTrack::AddTool(const std::string & toolName,
                                     const std::string & fileName,
                                     const bool isJson,
                                     const std::string & referenceName)
{
    // check if this tool already exists
    auto tool_iterator = m_tools.find(toolName);
    if (tool_iterator != m_tools.end()) {
        CMN_LOG_INIT_ERROR << "AddTool: " << toolName << " already exists" << std::endl;
        return false;
    }

    std::string _referenceName = referenceName;
    ToolsType::iterator referenceIterator;
    if (_referenceName != "") {
        referenceIterator = m_tools.find(_referenceName);
        if (referenceIterator == m_tools.end()) {
            CMN_LOG_INIT_ERROR << "AddTool: can't find reference \"" << _referenceName
                                     << "\" for tool \"" << toolName
                                     << "\".  Make sure reference frames/tools are created earlier in the config file."
                                     << std::endl;
            return false;
        }
    } else {
        _referenceName = this->Name;
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
        CMN_LOG_INIT_ERROR << "AddTool: failed to parse geometry file \""
                                 << fileName << "\" for tool \"" << toolName << "\"" << std::endl;
        return false;
    }

    ftkError error = ftkSetGeometry(m_internals->m_library, m_internals->m_sn, &geometry);
    if (error != FTK_ERROR_NS::FTK_OK) {
        CMN_LOG_INIT_ERROR << "AddTool: unable to set geometry for tool \"" << toolName
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
        CMN_LOG_INIT_ERROR << "AddTool: error, found an existing tool with the same Id "
                                 << geometry.geometryId << " for tool \"" << toolName << "\"" << std::endl;
        return false;
    }

    // finally create a cisst tool structure
    auto tool = new mtsAtracsysFusionTrackTool(toolName);

    // create an interface for tool
    tool->m_interface = AddInterfaceProvided(toolName);
    if (!tool->m_interface) {
        CMN_LOG_INIT_ERROR << "AddTool: " << tool->m_name << " already exists" << std::endl;
        delete tool;
        return false;
    }

    // register newly created tool
    CMN_LOG_CLASS_INIT_VERBOSE << "AddTool: registering tool " << toolName
                               << " with geometry Id " << geometry.geometryId << std::endl;
    this->m_tools[toolName] = tool;
    m_internals->GeometryIdToTool[geometry.geometryId] = tool;

    // frames used
    tool->m_local_measured_cp.ReferenceFrame() = this->Name;
    tool->m_local_measured_cp.MovingFrame() = toolName;
    tool->m_measured_cp.ReferenceFrame() = _referenceName;
    tool->m_measured_cp.MovingFrame() = toolName;
    if (_referenceName != this->Name) {
        tool->m_reference_measured_cp = &(referenceIterator->second->m_local_measured_cp);
    }

    // add data for this tool and populate tool interface
    tool->m_state_table.SetAutomaticAdvance(false);
    this->AddStateTable(&(tool->m_state_table));
    tool->m_state_table.AddData(tool->m_local_measured_cp, "local_measured_cp");
    tool->m_state_table.AddData(tool->m_measured_cp, "measured_cp");
    tool->m_state_table.AddData(tool->m_registration_error, "registration_error");
    tool->m_interface->AddCommandReadState(tool->m_state_table, tool->m_local_measured_cp, "local/measured_cp");
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


bool mtsAtracsysFusionTrack::HardwareInitialized() const
{
    return m_internals->m_configured;
}


void mtsAtracsysFusionTrack::ProcessIRTrackingFrame()
{
    ProcessTools();
    ProcessStrayMarkers();
}


void mtsAtracsysFusionTrack::ProcessRGBStereoFrame()
{
    if (!m_internals->m_images_enabled) {
        return;
    }

    // Image frames
    m_internals->ExtractLeftImage(m_left_image_raw);
    m_internals->ExtractRightImage(m_right_image_raw);
    m_left_image_raw.Valid() = true;
    m_right_image_raw.Valid() = true;
}


void mtsAtracsysFusionTrack::ProcessTools()
{
    // check results of last frame
    switch (m_internals->m_frame_query->markersStat) {
    case FTK_QS_NS::QS_WAR_SKIPPED:
        CMN_LOG_CLASS_RUN_ERROR << "Run: marker field is not written" << std::endl;
        break;
    case FTK_QS_NS::QS_ERR_INVALID_RESERVED_SIZE:
        CMN_LOG_CLASS_RUN_ERROR << "Run: frame.markersVersionSize is invalid" << std::endl;
        break;
    default:
        CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status" << std::endl;
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
        iter->second->m_local_measured_cp.SetValid(false);
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
            tool->m_local_measured_cp.SetValid(true);
            tool->m_local_measured_cp.Position().Translation().Assign(ft_tool.translationMM[0] * cmn_mm,
                                                                      ft_tool.translationMM[1] * cmn_mm,
                                                                      ft_tool.translationMM[2] * cmn_mm);
            for (size_t row = 0; row < 3; ++row) {
                for (size_t col = 0; col < 3; ++col) {
                    tool->m_local_measured_cp.Position().Rotation().Element(row, col) = ft_tool.rotation[row][col];
                }
            }
            tool->m_registration_error = ft_tool.registrationErrorMM * cmn_mm;
        }
    }

    // make pose relative to reference frame and finalize all tools
    for (iter = m_tools.begin(); iter != end; ++iter) {
        auto reference = iter->second->m_reference_measured_cp;
        if (reference == nullptr) {
            iter->second->m_measured_cp = iter->second->m_local_measured_cp;
        } else {
            // valid if both valid
            iter->second->m_measured_cp.Valid()
                = iter->second->m_local_measured_cp.Valid() && reference->Valid();
            // use reference pose
            iter->second->m_measured_cp.Position()
                = reference->Position().Inverse() * iter->second->m_local_measured_cp.Position();
        }
        iter->second->m_state_table.Advance();
    }
}


void mtsAtracsysFusionTrack::ProcessStrayMarkers()
{
    // ---- 3D Fiducials, aka stray markers ---
    switch (m_internals->m_frame_query->threeDFiducialsStat) {
    case FTK_QS_NS::QS_WAR_SKIPPED:
        CMN_LOG_CLASS_RUN_ERROR << "Stray markers: 3D fiducials fields not written" << std::endl;
        return;
    case FTK_QS_NS::QS_ERR_INVALID_RESERVED_SIZE:
        CMN_LOG_CLASS_RUN_ERROR << "Stray markers: frame.threeDFiducialsVersionSize must be multiple of fiducial type size" << std::endl;
        return;
    case FTK_QS_NS::QS_ERR_OVERFLOW:
        CMN_LOG_CLASS_RUN_ERROR << "Stray markers: too many stray markers detected --- try covering metallic/reflective surfaces" << std::endl;
        break;
    case FTK_QS_NS::QS_OK:
        break;
    default:
        int status = static_cast<int>(m_internals->m_frame_query->threeDFiducialsStat);
        CMN_LOG_CLASS_RUN_ERROR << "Run: invalid status " << status << std::endl;
        break;
    }

    size_t stray_count = m_internals->m_frame_query->threeDFiducialsCount;
    m_measured_cp_array.Positions().resize(stray_count);

    for (uint32 m = 0; m < stray_count; m++) {
        auto& marker_translation = m_measured_cp_array.Positions().at(m).Translation();
        marker_translation.Assign(m_internals->m_stray_markers[m].positionMM.x * cmn_mm,
                                  m_internals->m_stray_markers[m].positionMM.y * cmn_mm,
                                  m_internals->m_stray_markers[m].positionMM.z * cmn_mm);
    }
}
