/*
 * Copyright (c) 2014-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _NV_GST_CAPTURE_H_
#define _NV_GST_CAPTURE_H_

#include <gst/gst.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <math.h>

#include "mjpeg_server.h"
// #include "gst/pbutils/pbutils.h"
// #include "gst/pbutils/encoding-profile.h"
// #include "gst/pbutils/encoding-target.h"


/* CAPTURE GENERIC */
#define NVGST_DEFAULT_FILENAME                    "nvcamcap"
#define NVGST_DEFAULT_FILE_TYPE                   FILE_MP4
#define NVGST_DEFAULT_CAPTURE_FORMAT              "nv12"
#define NVGST_DEFAULT_CAPTURE_FPS                 30
#define DEFAULT_FILE_LOCATION                     "/dev/null"
#define SUCCESS                                   0

/* PREVIEW */
#define NVGST_DEFAULT_PREVIEW_WIDTH               1280
#define NVGST_DEFAULT_PREVIEW_HEIGHT              720
#define NVGST_DEFAULT_RENDER_TARGET               RENDER_OVERLAY

/* IMAGE & VIDEO CAPTURE */
#define NVGST_DEFAULT_VIDEO_MIMETYPE              "video/x-raw"
#define NVGST_DEFAULT_CAPTURE_WIDTH               1280
#define NVGST_DEFAULT_CAPTURE_HEIGHT              720
#define NVGST_DEFAULT_480P_ENCODER_BITRATE        4000000
#define NVGST_DEFAULT_720P_ENCODER_BITRATE        8000000
#define NVGST_DEFAULT_1080P_ENCODER_BITRATE       14000000
#define NVGST_DEFAULT_2160P_ENCODER_BITRATE       20000000
#define NVGST_DEFAULT_VIDEO_ENCODER_PROFILE       PROFILE_HIGH
#define NVGST_DEFAULT_VIDEO_ENCODER_CONTROLRATE   CONTROLRATE_VARIABLE
#define NVGST_DEFAULT_VIDEO_ENCODER_TWOPASSCBR    FALSE

#define NVGST_DEFAULT_IMAGE_ENCODER               FORMAT_JPEG_HW
#define NVGST_DEFAULT_VIDEO_ENCODER               FORMAT_H264_HW
#define NVGST_DEFAULT_FLIP_METHOD                 0

/* CAPTURE ELEMENTS */
#define NVGST_VIDEO_CAPTURE_SRC_TEST              "videotestsrc"
#define NVGST_VIDEO_CAPTURE_SRC_V4L2              "v4l2src"
#define NVGST_VIDEO_CAPTURE_SRC_CSI_ARGUS         "nvarguscamerasrc"
#define NVGST_EGLSTREAM_CAPTURE_SRC               "nveglstreamsrc"
#define NVGST_VIDEO_SINK                          "nvvideosink"
#define NVGST_DEFAULT_VIDEO_CONVERTER             "videoconvert"
#define NVGST_DEFAULT_VIDEO_CONVERTER_CSI         "nvvidconv"
#define NVGST_DEFAULT_VIDEO_SCALER                "videoscale"
#define NVGST_DEFAULT_PREVIEW_SINK_CSI            "nvoverlaysink"
#define NVGST_DEFAULT_PREVIEW_SINK_USB            "xvimagesink"
#define NVGST_DEFAULT_CAPTURE_FILTER              "capsfilter"
#define NVGST_DEFAULT_IMAGE_ENC                   "nvjpegenc"
#define NVGST_DEFAULT_IMAGE_ENC_CONVERTER         "nvvidconv"
#define NVGST_SW_IMAGE_ENC                        "jpegenc"
#define NVGST_DEFAULT_IENC_SINK                   "fakesink"
#define NVGST_DEFAULT_FILE_SINK                   "filesink"
#define NVGST_DEFAULT_VENC_PARSE                  "h264parse"
#define NVGST_PRIMARY_H264_VENC                   "omxh264enc"
#define NVGST_PRIMARY_VP8_VENC                    "omxvp8enc"
#define NVGST_PRIMARY_H265_VENC                   "omxh265enc"
#define NVGST_PRIMARY_VP9_VENC                    "omxvp9enc"
#define NVGST_PRIMARY_V4L2_H264_VENC              "nvv4l2h264enc"
#define NVGST_PRIMARY_V4L2_VP8_VENC               "nvv4l2vp8enc"
#define NVGST_PRIMARY_V4L2_VP9_VENC               "nvv4l2vp9enc"
#define NVGST_PRIMARY_V4L2_H265_VENC              "nvv4l2h265enc"
#define NVGST_PRIMARY_H264_PARSER                 "h264parse"
#define NVGST_PRIMARY_H265_PARSER                 "h265parse"
#define NVGST_PRIMARY_MP4_MUXER                   "qtmux"
#define NVGST_PRIMARY_3GP_MUXER                   "3gppmux"
#define NVGST_PRIMARY_MKV_MUXER                   "matroskamux"
#define NVGST_PRIMARY_STREAM_SELECTOR             "tee"
#define NVGST_PRIMARY_QUEUE                       "queue"
#define NVGST_PRIMARY_IDENTITY                    "identity"

/* CSI CAMERA DEFAULT PROPERTIES TUNING */

#define NVGST_DEFAULT_WHITEBALANCE                1
#define NVGST_DEFAULT_SATURATION                  1
#define NVGST_DEFAULT_EXPOSURE_COMPENSATION       0
#define NVGST_DEFAULT_TNR_STRENGTH                -1
#define NVGST_DEFAULT_EE_STRENGTH                 -1
#define NVGST_DEFAULT_AEANTIBANDING               0
#define NVGST_DEFAULT_AE_LOCK                     0
#define NVGST_DEFAULT_AWB_LOCK                    0
#define NVGST_DEFAULT_TNR_MODE                    1
#define NVGST_DEFAULT_EE_MODE                     1
#define NVGST_DEFAULT_SENSOR_ID                   0
#define NVGST_DEFAULT_SENSOR_MODE                 -1
#define NVGST_DEFAULT_DISPLAY_ID                  0

#define MIN_EXPOSURE_COMPENSATION                 -2
#define MAX_EXPOSURE_COMPENSATION                 2
#define MIN_TNR_MODE                              0
#define MAX_TNR_MODE                              2
#define MIN_EE_MODE                               0
#define MAX_EE_MODE                               2
#define MIN_STRENGTH                              -1
#define MAX_STRENGTH                              1
#define MIN_AE_ANTIBANDING_MODE                   0
#define MAX_AE_ANTIBANDING_MODE                   3

/* CSI CAMERA DEFAULT AUTOMATION */

#define NVGST_DEFAULT_AUTOMATION_MODE             FALSE
#define NVGST_DEFAULT_CAP_START_DELAY             5
#define NVGST_DEFAULT_QUIT_TIME                   0
#define NVGST_DEFAULT_ITERATION_COUNT             1
#define NVGST_DEFAULT_CAPTURE_GAP                 250
#define NVGST_DEFAULT_CAPTURE_TIME                10
#define NVGST_DEFAULT_NUM_SENSORS                 2
#define NVGST_DEFAULT_TOGGLE_CAMERA_MODE          FALSE
#define NVGST_DEFAULT_TOGGLE_CAMERA_SENSOR        FALSE
#define NVGST_DEFAULT_TOGGLE_CAMERA_SENSOR_MODES  FALSE
#define NVGST_DEFAULT_ENUMERATE_WHITEBALANCE      FALSE
#define NVGST_DEFAULT_ENUMERATE_SATURATION        FALSE
#define NVGST_DEFAULT_ENUMERATE_CAPTURE_AUTO      FALSE

#define MIN_V4L2_RES                              PR_640x480
#define MAX_V4L2_RES                              PR_1920x1080
#define MIN_CSI_RES                               PR_1280x720
#define MAX_CSI_RES                               PR_4032x3040

// /* DEBUG LOG LEVEL */
// #ifdef NVGST_LOG_LEVEL_DEBUG
// #define NVGST_ENTER_FUNCTION()            g_print("%s{", __FUNCTION__)
// #define NVGST_EXIT_FUNCTION()             g_print("%s}", __FUNCTION__)
// #define NVGST_EXIT_FUNCTION_VIA(s)        g_print("%s}['%s']", __FUNCTION__, s)
// #define NVGST_DEBUG_MESSAGE(s)            g_debug("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_DEBUG_MESSAGE_V(s, ...)     g_debug("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)
// #define NVGST_INFO_MESSAGE(s)             g_message("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_INFO_MESSAGE_V(s, ...)      g_message("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)
// #define NVGST_WARNING_MESSAGE(s)          g_warning("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_WARNING_MESSAGE_V(s, ...)   g_warning("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)
// #define NVGST_CRITICAL_MESSAGE(s)         do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_CRITICAL_MESSAGE_V(s, ...)  do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__,__VA_ARGS__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_ERROR_MESSAGE(s)            g_error("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_ERROR_MESSAGE_V(s, ...)     g_error("<%s:%d> "s, __FUNCTION__, __LINE__,__VA_ARGS__)

// #elif defined NVGST_LOG_LEVEL_INFO
// #define NVGST_ENTER_FUNCTION()            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION()             G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION_VIA(s)        G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE(s)            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE_V(s, ...)     G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_INFO_MESSAGE(s)             g_message("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_INFO_MESSAGE_V(s, ...)      g_message("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)
// #define NVGST_WARNING_MESSAGE(s)          g_warning("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_WARNING_MESSAGE_V(s, ...)   g_warning("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)
// #define NVGST_CRITICAL_MESSAGE(s)         do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_CRITICAL_MESSAGE_V(s, ...)  do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_ERROR_MESSAGE(s)            g_error("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_ERROR_MESSAGE_V(s, ...)     g_error("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)

// #elif defined NVGST_LOG_LEVEL_WARNING
// #define NVGST_ENTER_FUNCTION()            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION()             G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION_VIA(s)        G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE(s)            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE_V(s, ...)     G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_INFO_MESSAGE(s)             G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_INFO_MESSAGE_V(s, ...)      G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_WARNING_MESSAGE(s)          g_warning("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_WARNING_MESSAGE_V(s, ...)   g_warning("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)
// #define NVGST_CRITICAL_MESSAGE(s)         do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_CRITICAL_MESSAGE_V(s, ...)  do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_ERROR_MESSAGE(s)            g_error("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_ERROR_MESSAGE_V(s, ...)     g_error("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)

// #elif defined NVGST_LOG_LEVEL_CRITICAL
// #define NVGST_ENTER_FUNCTION()            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION()             G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION_VIA(s)        G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE(s)            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE_V(s, ...)     G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_INFO_MESSAGE(s)             G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_INFO_MESSAGE_V(s, ...)      G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_WARNING_MESSAGE(s)          G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_WARNING_MESSAGE_V(s, ...)   G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_CRITICAL_MESSAGE(s)         do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_CRITICAL_MESSAGE_V(s, ...)  do {\
//                                           g_critical("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__);\
//                                           app->return_value = -1;\
//                                           } while (0)
// #define NVGST_ERROR_MESSAGE(s)            g_error("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_ERROR_MESSAGE_V(s, ...)     g_error("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)

// #else
// #define NVGST_ENTER_FUNCTION()            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION()             G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_EXIT_FUNCTION_VIA(s)        G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE(s)            G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_DEBUG_MESSAGE_V(s, ...)     G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_INFO_MESSAGE(s)             G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_INFO_MESSAGE_V(s, ...)      G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_WARNING_MESSAGE(s)          G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_WARNING_MESSAGE_V(s, ...)   G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_CRITICAL_MESSAGE(s)         G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_CRITICAL_MESSAGE_V(s, ...)  G_STMT_START{ (void)0; }G_STMT_END
// #define NVGST_ERROR_MESSAGE(s)            g_error("<%s:%d> "s, __FUNCTION__, __LINE__)
// #define NVGST_ERROR_MESSAGE_V(s, ...)     g_error("<%s:%d> "s, __FUNCTION__, __LINE__, __VA_ARGS__)
// #endif

#define NVGST_ENTER_FUNCTION()            G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_EXIT_FUNCTION()             G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_EXIT_FUNCTION_VIA(s)        G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_DEBUG_MESSAGE(s)            G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_DEBUG_MESSAGE_V(s, ...)     G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_INFO_MESSAGE(s)             G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_INFO_MESSAGE_V(s, ...)      G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_WARNING_MESSAGE(s)          G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_WARNING_MESSAGE_V(s, ...)   G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_CRITICAL_MESSAGE(s)         G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_CRITICAL_MESSAGE_V(s, ...)  G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_ERROR_MESSAGE(s)            G_STMT_START{ (void)0; }G_STMT_END
#define NVGST_ERROR_MESSAGE_V(s, ...)     G_STMT_START{ (void)0; }G_STMT_END

#define INVALID_SELECTION_ARGUS                    "Not a valid option for ARGUS Plugin\n"

/* CAMERA KPI PARAMS */
typedef enum
{
  FIRST_FRAME = 0,
  APP_LAUNCH,
  CURRENT_EVENT,
  KPI_EVENT_SIZE,
} KpiEvents;

/* CAMERA AUTOMATION PARAMS */
typedef struct
{
  gint capture_start_time;
  gint quit_time;
  gint iteration_count;
  gint capture_gap;
  gint capture_time;
  gint num_sensors;
  gboolean automate;
  gboolean toggle_mode;
  gboolean toggle_sensor;
  gboolean toggle_sensor_modes;
  gboolean enum_wb;
  gboolean enum_st;
  gboolean capture_auto;
} Automate;

/* PREVIEW RESOLUTION */
typedef enum
{
  PR_640x480 = 0,
  PR_1280x720,
  PR_1920x1080,
  PR_3840x2160,
  PR_3264x1848,
  PR_3264x2464,
  PR_2560x1440,
  PR_2592x1944,
  PR_4032x3040,
} Prev_Res;

/* IMAGE CAPTURE RESOLUTION */
typedef enum
{
  IR_640x480 = 0,
  IR_1280x720,
  IR_1920x1080,
  IR_3840x2160,
  IR_3264x1848,
  IR_3264x2464,
  IR_2560x1440,
  IR_2592x1944,
  IR_4032x3040,
} Icap_Res;

/* VIDEO CAPTURE RESOLUTION */
typedef enum
{
  VR_640x480 = 0,
  VR_1280x720,
  VR_1920x1080,
  VR_3840x2160,
  VR_3264x1848,
  VR_3264x2464,
  VR_2560x1440,
  VR_2592x1944,
  VR_4032x3040,
} Vcap_Res;

#define RESOLUTION_STRINGS {"640 x 480", "1280 x 720", \
  "1920 x 1080", "3840 x 2160", "3264 x 1848", "3264 x 2464", \
  "2560 x 1440", "2592 x 1944", "4032 x 3040", NULL};

/* CAPTURE CONTAINER TYPE */
typedef enum
{
  FILE_MP4 = 0,
  FILE_3GP,
  FILE_MKV,
  FILE_H265
} FileType;

#define FILE_TYPE_STRINGS {"MP4", "3GP", "MKV", "H.265", NULL};

/* IMAGE ENCODE TYPE */
typedef enum
{
  FORMAT_JPEG_SW = 0,
  FORMAT_JPEG_HW
} ImageEncFormatType;

#define IMAGE_ENCODER_STRINGS {"SW JPEG", "HW JPEG", NULL};

/* VIDEO ENCODE TYPE */
typedef enum
{
  FORMAT_H264_HW = 0,
  FORMAT_VP8_HW,
  FORMAT_H265_HW,
  FORMAT_VP9_HW
} VideoEncFormatType;

#define VIDEO_ENC_STRINGS {"H.264 (HW)", "VP8 (HW)", "H.265 (HW)", "VP9 (HW)", NULL};

/* H264 ENCODE PROFILE TYPE */
typedef enum
{
  PROFILE_BASELINE = 0,
  PROFILE_MAIN,
  PROFILE_HIGH
} H264EncProfileType;

/* ENCODER BITRATE CONTROL METHOD */
typedef enum
{
  CONTROLRATE_DISABLE,
  CONTROLRATE_VARIABLE,
  CONTROLRATE_CONSTANT
} EncControlRateType;

/* CAPTURE MODE */
typedef enum
{
  CAPTURE_NONE = 0,
  CAPTURE_IMAGE,
  CAPTURE_VIDEO
} CaptureType;

/* CAPTURE COLOR FORMAT */
typedef enum
{
  CAPTURE_I420,
  CAPTURE_NV12,
  CAPTURE_YUY2
} CaptureColorFormat;

/* CAPTURE PAD TYPE */
typedef enum
{
  CAPTURE_PAD_PREV = 0,
  CAPTURE_PAD_IMAGE,
  CAPTURE_PAD_VIDEO
} CapturePadType;

typedef enum
{
  NV_CAM_SRC_V4L2,
  NV_CAM_SRC_CSI,
  NV_CAM_SRC_TEST,
  NV_CAM_SRC_EGLSTREAM
} NvCamSrcType;

typedef enum
{
  HW_OMX_ENC = 0,
  HW_V4L2_ENC
} HardwareEncoderType;

// CAMERA CAPTURE RESOLUTIONS 
typedef struct
{
  gint image_cap_width;
  gint image_cap_height;
  gint img_res_index;
  gint video_cap_width;
  gint video_cap_height;
  gint vid_res_index;
  gint current_max_res;
} CamRes;

// CAMERA ENCODER PARAMS 
typedef struct
{
  gint image_enc;
  gint video_enc;
  guint bitrate;
  gboolean enabletwopassCBR;
  EncControlRateType controlrate;
  H264EncProfileType video_enc_profile;
} EncSet;

// CAMERA PIPELINE 
typedef struct 
{
  GstElement *cap_bin;
  GstElement *vsrc;
  GstElement *cap_filter;

  GstElement *cap_tee;
  GstElement *venc_q;
  GstElement *ienc_q;

  GstElement *vid_conv_bin;
  GstElement *vconv;
  GstElement *vconv_out_filter;

  GstElement *img_conv_bin;
  GstElement *iconv;
  GstElement *iconv_out_filter;

  GstElement *vid_enc_bin;
  GstElement *venc;
  GstElement *parser;

  GstElement *img_enc_bin;
  GstElement *ienc;
  GstElement *fake_sink;
} CamPipe; 

#define MAX_NUM_CAMS 2

// GSTREAMER PIPELINE 
typedef struct
{
  GstElement *pipeline;
  CamPipe cams[MAX_NUM_CAMS]; 

  GstElement *vid_sink_bin;
  GstElement *muxer;
  GstElement *file_sink;
} CapPipe;

typedef struct 
{
  gint whitebalance;
  gint ae_antibanding;
  gint tnr_mode;
  gint ee_mode;
  gint timeout;

  gfloat saturation;
  gfloat exposure_compensation;
  gfloat tnr_strength;
  gfloat ee_strength;
  guint sensor_id;
  guint sensor_mode;
  guint flip_method;

  gboolean enableAeLock;
  gboolean enableAwbLock;

  gchar *exposure_timerange;
  gchar *gain_range;
  gchar *isp_digital_gainrange;

} CamSet; 

typedef struct
{
  gint return_value;
  
  guint num_cams; 
  guint cam_ids[MAX_NUM_CAMS];

  gint file_type;
  gchar *file_name;
  gboolean muxer_is_identity;
  GMutex *lock;
  GCond *cond;
  GThread *reset_thread;

  CamRes capres;
  EncSet encset;
  CamSet camsets[MAX_NUM_CAMS]; 

  CapPipe cap;
  gulong venc_probe_id;
  gulong ienc_probe_id;

  mjpeg_server *stream_server; 
} AppCtx;

#endif
