/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2024-07-01

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsAtracsysStereo_h
#define _mtsAtracsysStereo_h

#include <string>

#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmImageFrame.h>
#include <cisstParameterTypes/prmCameraInfo.h>
#include <cisstParameterTypes/prmDepthMap.h>

#include <json/json.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/stereo.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include <sawAtracsysFusionTrack/sawAtracsysFusionTrackExport.h>  // always include last

class CISST_EXPORT mtsAtracsysStereo: public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    inline mtsAtracsysStereo(const std::string & componentName, std::string reference_frame):
        mtsTaskContinuous(componentName, 100),
        m_reference_frame(reference_frame) {
        Init();
    }

    inline mtsAtracsysStereo(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg),
        m_reference_frame(GetName()) {
        Init();
    }

    ~mtsAtracsysStereo(void) {};

    void Configure(const std::string & filename = "") override;

    void Startup(void) override;

    void Run(void) override;

    void Cleanup(void) override;

protected:
    void Init(void); // shared initialization by all constructors
    void InitStereoPipeline(); // one-time pipeline resource initialization
    void Rectify(prmImageFrame& raw, cv::Mat& rectified_mat, prmImageFrame& rectified, const cv::Mat& undistort_x, const cv::Mat& undistort_y);
    void ComputeDepth();

    std::string m_reference_frame;

    mtsInterfaceProvided * m_stereo_interface = nullptr;
    mtsInterfaceRequired * m_stereo_raw_interface = nullptr;

    prmCameraInfo m_left_camera_info;
    prmImageFrame m_left_image;
    prmCameraInfo m_right_camera_info;
    prmImageFrame m_right_image;
    prmDepthMap m_depth;

    mtsFunctionRead m_get_left_image_raw;
    mtsFunctionRead m_get_right_image_raw;
    mtsFunctionRead m_get_left_camera_info;
    mtsFunctionRead m_get_right_camera_info;
    mtsFunctionRead m_get_camera_rotation;
    mtsFunctionRead m_get_camera_translation;

    cv::Mat m_left_image_mat, m_right_image_mat;

    bool m_pipline_initialized;
    bool m_video_enabled;
    bool m_depth_enabled;
    bool m_color_pointcloud;

    bool m_global_block_matching;
    bool m_filter_depth_map;
    double m_min_depth;

    cv::Mat m_left_undistort_map_x, m_left_undistort_map_y;
    cv::Mat m_right_undistort_map_x, m_right_undistort_map_y;
    cv::Mat disparity_to_depth;

    cv::Ptr<cv::StereoMatcher> m_left_stereo_matcher, m_right_stereo_matcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> m_wls_filter;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsAtracsysStereo);

#endif  // _mtsAtracsysStereo_h
