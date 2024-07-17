/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-17

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawAtracsysFusionTrack/mtsAtracsysStereo.h>
#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackConfig.h>

#include <limits>

#include <ftkInterface.h>

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <opencv2/calib3d.hpp>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAtracsysStereo, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

void mtsAtracsysStereo::Init(void)
{
    StateTable.AddData(m_left_image, "left_image");
    StateTable.AddData(m_right_image, "right_image");
    StateTable.AddData(m_left_camera_info, "left_camera_info");
    StateTable.AddData(m_right_camera_info, "right_camera_info");
    StateTable.AddData(m_depth, "pointcloud");

    m_stereo_interface = AddInterfaceProvided("stereo");
    if (m_stereo_interface) {
        m_stereo_interface->AddCommandReadState(StateTable, m_left_image, "left/image_rect_color");
        m_stereo_interface->AddCommandReadState(StateTable, m_left_camera_info, "left/camera_info");
        m_stereo_interface->AddCommandReadState(StateTable, m_right_image, "right/image_rect_color");
        m_stereo_interface->AddCommandReadState(StateTable, m_right_camera_info, "right/camera_info");
        m_stereo_interface->AddCommandReadState(StateTable, m_depth, "pointcloud");

        m_stereo_interface->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");
    }

    m_stereo_raw_interface = AddInterfaceRequired("StereoRaw");
    if (m_stereo_raw_interface) {
        m_stereo_raw_interface->AddFunction("left/image_raw", m_get_left_image_raw);
        m_stereo_raw_interface->AddFunction("right/image_raw", m_get_right_image_raw);
        m_stereo_raw_interface->AddFunction("left/camera_info", m_get_left_camera_info);
        m_stereo_raw_interface->AddFunction("right/camera_info", m_get_right_camera_info);
        m_stereo_raw_interface->AddFunction("camera_rotation", m_get_camera_rotation);
        m_stereo_raw_interface->AddFunction("camera_translation", m_get_camera_translation);
    }

    m_pipline_initialized = false;
    m_video_enabled = false;
    m_depth_enabled = false;
    m_color_pointcloud = false;

    m_global_block_matching = false;
    m_filter_depth_map = false;
}

void mtsAtracsysStereo::InitStereoPipeline()
{
    m_pipline_initialized = false;

    mtsExecutionResult result;

    /// Retrieve stereo calibration

    result = m_get_left_camera_info(m_left_camera_info);
    if (!result.IsOK() || !m_left_camera_info.Valid()) {
        return;
    }

    result = m_get_right_camera_info(m_right_camera_info);
    if (!result.IsOK() || !m_right_camera_info.Valid()) {
        return;
    }

    vct3 rotation, translation;

    result = m_get_camera_rotation(rotation);
    if (!result.IsOK()) {
        return;
    }

    result = m_get_camera_translation(translation);
    if (!result.IsOK()) {
        return;
    }

    /// Convert to OpenCV calibration format

    cv::Mat left_intrinsic = cv::Mat(3, 3, CV_64F, m_left_camera_info.Intrinsic().Pointer());
    cv::Mat right_intrinsic = cv::Mat(3, 3, CV_64F, m_right_camera_info.Intrinsic().Pointer());
    cv::Mat left_distortion = cv::Mat(5, 1, CV_64F, m_left_camera_info.DistortionParameters().Pointer());
    cv::Mat right_distortion = cv::Mat(5, 1, CV_64F, m_right_camera_info.DistortionParameters().Pointer());

    cv::Size original_image_size = cv::Size(m_left_camera_info.Width(), m_left_camera_info.Height());

    cv::Mat R = cv::Mat(3, 1, CV_64F, rotation.Pointer());
    cv::Mat T = cv::Mat(3, 1, CV_64F, translation.Pointer());

    // stereo rectification output matrices
    cv::Mat left_rect, right_rect;
    cv::Mat left_proj, right_proj;

    // Compute rectification maps and projection matrices

    cv::stereoRectify(
        left_intrinsic, left_distortion,
        right_intrinsic, right_distortion,
        original_image_size, R, T,
        left_rect, right_rect, left_proj, right_proj,
        disparity_to_depth
    );

    cv::initUndistortRectifyMap(
        left_intrinsic, left_distortion,
        left_rect, left_proj,
        original_image_size,
        CV_32FC(1),
        m_left_undistort_map_x, m_left_undistort_map_y
    );

    cv::initUndistortRectifyMap(
        right_intrinsic, right_distortion,
        right_rect, right_proj,
        original_image_size,
        CV_32FC(1),
        m_right_undistort_map_x, m_right_undistort_map_y
    );

    /// Convert results to CISST data types

    std::copy(left_rect.begin<double>(), left_rect.end<double>(), m_left_camera_info.Rectification().begin());
    std::copy(right_rect.begin<double>(), right_rect.end<double>(), m_right_camera_info.Rectification().begin());
    std::copy(left_proj.begin<double>(), left_proj.end<double>(), m_left_camera_info.Projection().begin());
    std::copy(right_proj.begin<double>(), right_proj.end<double>(), m_right_camera_info.Projection().begin());

    /// Create stereo matcher(s) and depth map filters

    int block_size = 25;
    int block_area = block_size * block_size;
    if (!m_global_block_matching) {
        m_left_stereo_matcher = cv::StereoBM::create(384, block_size);
    } else {
        m_left_stereo_matcher = cv::StereoSGBM::create(0, 384, block_size, 8 * block_area, 28 * block_area, 0, 0, 10, 100, 1, cv::StereoSGBM::MODE_SGBM);
    }

    m_right_stereo_matcher = cv::ximgproc::createRightMatcher(m_left_stereo_matcher);
    m_wls_filter = cv::ximgproc::createDisparityWLSFilter(m_left_stereo_matcher);
    m_wls_filter->setLambda(8000.0);
    m_wls_filter->setSigmaColor(1.0);

    m_pipline_initialized = true;
}

void mtsAtracsysStereo::Rectify(prmImageFrame& raw, cv::Mat& rectified_mat, prmImageFrame& rectified, const cv::Mat& undistort_x, const cv::Mat& undistort_y)
{
    /// De-bayer image
    cv::Mat raw_bayer = cv::Mat(raw.Height(), raw.Width(), CV_8U, raw.Data().Pointer());
    cv::Mat raw_rgb;
    cv::cvtColor(raw_bayer, raw_rgb, cv::COLOR_BayerGR2BGR_EA);

    // Rectify
    cv::remap(raw_rgb, rectified_mat, undistort_x, undistort_y, cv::INTER_LINEAR);

    rectified.Width() = rectified_mat.cols;
    rectified.Height() = rectified_mat.rows;
    rectified.Channels() = rectified_mat.channels();

    // Convert to prmImageFrame
    size_t size = rectified_mat.cols * rectified_mat.rows * rectified_mat.channels();
    rectified.Data().resize(size);
    std::copy(rectified_mat.data, rectified_mat.data + size, rectified.Data().begin());
}

void mtsAtracsysStereo::ComputeDepth()
{
    // Convert to gray-scale
    cv::Mat left_gray, right_gray;
    cv::cvtColor(m_left_image_mat, left_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(m_right_image_mat, right_gray, cv::COLOR_RGB2GRAY);

    // Compute left-to-right stereo correspondence
    cv::Mat left_disparity, right_disparity;
    m_left_stereo_matcher->compute(left_gray, right_gray, left_disparity);

    // If filtering is enabled, compute the right-to-left correspondence
    // and run WLS filter
    cv::Mat disparity;
    if (m_filter_depth_map) {
        m_right_stereo_matcher->compute(right_gray, left_gray, right_disparity);
        m_wls_filter->filter(left_disparity, m_left_image_mat, disparity, right_disparity, cv::Rect(), m_right_image_mat);
    } else {
        disparity = left_disparity;
    }
    
    // Disparity maps use fixed-point format, need to rescale by 1/DISP_SCALE (1/16) to convert to floating point
    disparity.convertTo(disparity, CV_32F, 1.0f/cv::StereoMatcher::DISP_SCALE, 0.0f);

    // Use stereo calibration to convert from disparity to depth values
    cv::Mat depth;
    cv::reprojectImageTo3D(disparity, depth, disparity_to_depth, true, -1);

    float opencv_bigz = 10000.0f;
    float invalid = std::numeric_limits<float>::quiet_NaN();
    cv::Mat z;
    cv::extractChannel(depth, z, 2);
    cv::Mat mask = (z<=0.0f) | (z==opencv_bigz);
    depth.setTo(cv::Scalar(invalid, invalid, invalid), mask);

    m_depth.Width() = depth.cols;
    m_depth.Height() = depth.rows;

    size_t size = depth.cols * depth.rows * depth.channels();

    m_depth.Points().resize(size);
    std::copy(reinterpret_cast<float*>(depth.data), reinterpret_cast<float*>(depth.data) + size, m_depth.Points().begin());

    if (m_color_pointcloud) {
        m_depth.Color().resize(m_left_image_mat.cols * m_left_image_mat.rows * m_left_image_mat.channels());
        std::copy(m_left_image_mat.data, m_left_image_mat.data + size, m_depth.Color().begin());
    } else {
        m_depth.Color().resize(0);
    }
}

void mtsAtracsysStereo::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    cmnPath m_config_path;
    m_config_path.Add(cmnPath::GetWorkingDirectory());
    m_config_path.Add(std::string(sawAtracsysFusionTrack_SOURCE_DIR) + "/../share", cmnPath::TAIL);

    std::string config_fullname = m_config_path.Find(filename);
    if (!cmnPath::Exists(config_fullname)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: configuration file \"" << filename
                                 << "\" not found in path (" << m_config_path << ")" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::ifstream jsonStream;
    jsonStream.open(config_fullname.c_str());

    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
        return;
    }

    if (jsonConfig.isMember("stereo")) {
        const Json::Value stereo = jsonConfig["stereo"];
        const Json::Value video = stereo["video"];
        m_video_enabled = stereo.isMember("video") && video.isBool() && video.asBool();
        const Json::Value depth = stereo["depth"];
        m_depth_enabled = stereo.isMember("depth") && depth.isBool() && depth.asBool();

        // video processing is required for depth map computation
        m_video_enabled = m_depth_enabled || m_video_enabled;

        const Json::Value color_pointcloud = stereo["color_pointcloud"];
        m_color_pointcloud = stereo.isMember("color_pointcloud") && color_pointcloud.isBool() && color_pointcloud.asBool();

        const Json::Value global_bm = stereo["global_block_matching"];
        m_global_block_matching = stereo.isMember("global_block_matching") && global_bm.isBool() && global_bm.asBool();

        const Json::Value filter_depth = stereo["filter_depth_map"];
        m_filter_depth_map = stereo.isMember("filter_depth_map") && filter_depth.isBool() && filter_depth.asBool();
    }
}

void mtsAtracsysStereo::Startup(void) { }

void mtsAtracsysStereo::Run(void)
{
    // process mts commands
    ProcessQueuedCommands();

    if (!m_depth_enabled && !m_video_enabled) {
        return;
    }

    if (!m_pipline_initialized) {
        InitStereoPipeline();
        return;
    }

    mtsExecutionResult result;
    prmImageFrame left_raw, right_raw;

    result = m_get_left_image_raw(left_raw);
    if (!result.IsOK() || !left_raw.Valid()) {
        return;
    }

    result = m_get_right_image_raw(right_raw);
    if (!result.IsOK() || !right_raw.Valid()) {
        return;
    }

    Rectify(left_raw, m_left_image_mat, m_left_image, m_left_undistort_map_x, m_left_undistort_map_y);
    Rectify(right_raw, m_right_image_mat, m_right_image, m_right_undistort_map_x, m_right_undistort_map_y);
    m_left_image.Valid() = true;
    m_right_image.Valid() = true;

    if (!m_depth_enabled) {
        return;
    }

    ComputeDepth();
    m_depth.Valid() = true;
    m_depth.ReferenceFrame() = m_reference_frame;
}

void mtsAtracsysStereo::Cleanup() {}
