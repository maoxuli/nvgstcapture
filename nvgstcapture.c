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

#include <dlfcn.h>
#include <unistd.h>
#include "nvgstcapture.h"

#define EGL_PRODUCER_LIBRARY "libnveglstreamproducer.so"

#ifdef WITH_STREAMING
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include "gstnvrtspserver.h"
static NvGstRtspFunctions nvgst_rtsp_functions;



static GstFlowReturn
rtsp_video_appsink_new_sample (GstAppSink * appsink, gpointer user_data);
#endif

static gboolean check_capture_params (void);
gboolean create_capture_pipeline (void);
static gboolean create_native_capture_pipeline (void);
void destroy_capture_pipeline (void);
static void capture_init_params (void);
static void set_encoder_bitrate (guint bitrate);
static void set_encoder_profile (H264EncProfileType profile);
void set_capture_device_node (void);
static void print_help (void);
static void set_new_file_name (int muxer_type);
void restart_capture_pipeline (void);
static gboolean create_svs_bin (void);
static gboolean create_cap_bin (void);
static gboolean create_vid_enc_bin (void);
static gboolean create_img_enc_bin (void);

static gboolean get_image_encoder (GstElement ** iencoder);
static gboolean get_video_encoder (GstElement ** vencoder);
static gboolean get_muxer (GstElement ** muxer);
static void
cam_image_captured (GstElement * fsink,
    GstBuffer * buffer, GstPad * pad, gpointer udata);
static gboolean
parse_spec (const gchar * option_name,
    const gchar * value, gpointer data, GError ** error);
static GstPadProbeReturn
prev_buf_prob (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
static GstPadProbeReturn
enc_buf_prob (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
gboolean get_preview_resolution (gint res);
static gboolean get_image_capture_resolution (gint res);
static gboolean get_video_capture_resolution (gint res);
static gboolean camera_need_reconfigure (int new_res,
    CapturePadType current_pad);

static AppCtx appctx;
AppCtx *app = NULL;

static GMainLoop *loop = NULL;
gboolean recording = FALSE;
static gboolean snapshot = FALSE;

/* EGLStream Producer */
typedef gint (*start_eglstream_producer_func)
  (int producer_index, EGLDisplay * display, EGLStreamKHR * stream,
   int width, int height);
typedef gint (*stop_eglstream_producer_func) (int producer_index);

static int is_user_bitrate = 0;

void set_saturation (gfloat dval);
void set_whitebalance (gint val);
void set_timeout(gint val);
void set_mode (gint newMode);
void set_exposure_saturation (gfloat dval);

gboolean set_preview_resolution (int new_res);
gboolean set_image_resolution (int new_res);
gboolean set_video_resolution (int new_res);

void start_video_capture (void);
void stop_video_capture (void);
void trigger_vsnap_capture (void);
void trigger_image_capture (void);
gboolean exit_capture (gpointer data);

#if !GUI
static void nvgst_handle_xevents (void);
static gpointer nvgst_x_event_thread (gpointer);
#endif

#define WIDTH_RES   640, 1280, 1920, 3264, 3264, 2560, 2592, 4032
#define HEIGHT_RES  480, 720,  1080, 1848, 2464, 1440, 1944, 3040

gint prevres_width[] = { WIDTH_RES };
gint prevres_height[] = { HEIGHT_RES };
static gint image_capture_width[] = { WIDTH_RES };
static gint image_capture_height[] = { HEIGHT_RES };
static gint video_capture_width[] = { WIDTH_RES };
static gint video_capture_height[] = { HEIGHT_RES };


static gboolean cintr = FALSE;

static gboolean check_for_interrupt (gpointer data)
{
  if (cintr) {
    cintr = FALSE;

    gst_element_post_message (GST_ELEMENT (app->pipeline.pipeline),
        gst_message_new_application (GST_OBJECT (app->pipeline.pipeline),
            gst_structure_new ("NvGstAppInterrupt",
                "message", G_TYPE_STRING, "Pipeline interrupted", NULL)));

    return FALSE;
  }
  return TRUE;
}

static void _intr_handler (int signum)
{
    struct sigaction action;
    NVGST_INFO_MESSAGE ("User Interrupted..");
    app->return_value = -1;
    memset (&action, 0, sizeof (action));
    action.sa_handler = SIG_DFL;
    sigaction (SIGINT, &action, NULL);
    cintr = TRUE;
}

static void _intr_setup (void)
{
    struct sigaction action;
    memset (&action, 0, sizeof (action));
    action.sa_handler = _intr_handler;
    sigaction (SIGINT, &action, NULL);
}


/**
  * get the max capture resolutions
  *
  * @param res : resolution index
  */
static void
get_max_resolution (gint res, gint * width, gint * height)
{
  if (app->use_cus_res) {
    *width = app->capres.cus_prev_width;
    *height = app->capres.cus_prev_height;
  } else {
    *width = image_capture_width[res];
    *height = image_capture_height[res];
  }
}

static gboolean
get_image_capture_resolution (gint res)
{
  gboolean ret = TRUE;

  if ( (app->cam_src == NV_CAM_SRC_CSI) ||
      (app->cam_src == NV_CAM_SRC_EGLSTREAM) ) {
    if ((res < IR_1280x720) || (res > IR_4032x3040)) {
      g_print ("Invalid image capture resolution\n");
      return FALSE;
    }
  } else {
    if ((res < IR_640x480) || (res > IR_1920x1080)) {
      g_print ("Invalid image capture resolution\n");
      return FALSE;
    }
  }
  app->capres.image_cap_width = image_capture_width[res];
  app->capres.image_cap_height = image_capture_height[res];
  app->capres.img_res_index = res;

  return ret;
}

static gboolean
get_video_capture_resolution (gint res)
{
  gboolean ret = TRUE;

  if ( (app->cam_src == NV_CAM_SRC_CSI) ||
      (app->cam_src == NV_CAM_SRC_EGLSTREAM) ){
    if ((res < VR_1280x720) || (res > VR_4032x3040)) {
      g_print ("Invalid video capture resolution\n");
      return FALSE;
    }
  } else {
    if ((res < VR_640x480) || (res > VR_1280x720)) {
      g_print ("Invalid video capture resolution\n");
      return FALSE;
    }
  }
  app->capres.video_cap_width = video_capture_width[res];
  app->capres.video_cap_height = video_capture_height[res];
  app->capres.vid_res_index = res;

  return ret;
}

static gpointer
reset_elements (gpointer data)
{
  gst_element_set_state (app->ele.venc_q, GST_STATE_READY);
  gst_element_set_state (app->ele.vid_bin, GST_STATE_READY);
  gst_element_set_state (app->ele.svc_vidbin, GST_STATE_READY);

  gst_element_sync_state_with_parent (app->ele.venc_q);
  gst_element_sync_state_with_parent (app->ele.vid_bin);
  gst_element_sync_state_with_parent (app->ele.svc_vidbin);

  return NULL;
}


void
set_mode (gint newMode)
{
  if (newMode != 1 && newMode != 2) {
    newMode = NVGST_DEFAULT_CAPTURE_MODE;
    g_print ("Invalid input mode, setting mode to image-capture = 1 \n");
  }
  g_print ("Changing capture mode to %d\n", newMode);
  g_print ("(1): image\n(2): video\n");

  if (app->cam_src == NV_CAM_SRC_CSI) {
    g_object_set (app->ele.cap_tee, "mode", newMode, NULL);
  } else {
    destroy_capture_pipeline ();
    g_usleep (250000);
    app->mode = newMode;
    if (!create_capture_pipeline ()) {
      app->return_value = -1;
      g_main_loop_quit (loop);
    }
  }
  app->mode = newMode;
}

gboolean
set_preview_resolution (int new_res)
{
  GstCaps *caps = NULL;
  gint width = 0, height = 0;
  if (new_res == app->capres.prev_res_index) {
    g_print ("\nAlready on same preview resolution\n");
    return TRUE;
  }
  if (!get_preview_resolution (new_res))
    return FALSE;

  g_object_get (app->ele.svc_prevconv_out_filter, "caps", &caps, NULL);
  caps = gst_caps_make_writable (caps);

  gst_caps_set_simple (caps, "width", G_TYPE_INT,
      app->capres.preview_width, "height", G_TYPE_INT,
      app->capres.preview_height, NULL);

  g_object_set (app->ele.svc_prevconv_out_filter, "caps", caps, NULL);
  gst_caps_unref (caps);

  if (camera_need_reconfigure (new_res, CAPTURE_PAD_PREV)) {
    g_object_get (app->ele.cap_filter, "caps", &caps, NULL);
    caps = gst_caps_make_writable (caps);

    get_max_resolution (app->capres.current_max_res, &width, &height);
    gst_caps_set_simple (caps, "width", G_TYPE_INT,
        width, "height", G_TYPE_INT, height, NULL);

    g_object_set (app->ele.cap_filter, "caps", caps, NULL);
    gst_caps_unref (caps);
  }

#if !GUI
{
  GstElement *vsink = app->ele.vsink;

  if (vsink && GST_IS_VIDEO_OVERLAY (vsink)) {
    if (app->capres.preview_width < app->disp.display_width
        || app->capres.preview_height < app->disp.display_height) {
      app->disp.width = app->capres.preview_width;
      app->disp.height = app->capres.preview_height;
    } else {
      app->disp.width = app->disp.display_width;
      app->disp.height = app->disp.display_height;
    }
    g_mutex_lock (app->lock);

    if (app->disp.window)
      nvgst_destroy_window (&app->disp);
    nvgst_create_window (&app->disp, "nvgstcapture");
    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (vsink),
        (gulong) app->disp.window);
    gst_video_overlay_expose (GST_VIDEO_OVERLAY (vsink));

    g_mutex_unlock (app->lock);
 }
}
#endif
  g_print ("Preview resolution = %d x %d\n",
      app->capres.preview_width, app->capres.preview_height);

  return TRUE;
}

gboolean
set_image_resolution (int new_res)
{
  GstCaps *caps = NULL;
  gint width = 0, height = 0;
  if (new_res == app->capres.img_res_index) {
    g_print ("\nAlready on same image capture resolution\n");
    return TRUE;
  }
  if (!get_image_capture_resolution (new_res))
    return FALSE;

  //configure image
  g_object_get (app->ele.svc_imgvconv_out_filter, "caps", &caps, NULL);
  caps = gst_caps_make_writable (caps);

  gst_caps_set_simple (caps, "width", G_TYPE_INT,
      app->capres.image_cap_width, "height", G_TYPE_INT,
      app->capres.image_cap_height, NULL);

  g_object_set (app->ele.svc_imgvconv_out_filter, "caps", caps, NULL);
  gst_caps_unref (caps);

  if (camera_need_reconfigure (new_res, CAPTURE_PAD_IMAGE)) {
    g_object_get (app->ele.cap_filter, "caps", &caps, NULL);
    caps = gst_caps_make_writable (caps);

    get_max_resolution (app->capres.current_max_res, &width, &height);
    gst_caps_set_simple (caps, "width", G_TYPE_INT,
        width, "height", G_TYPE_INT, height, NULL);

    g_object_set (app->ele.cap_filter, "caps", caps, NULL);
    gst_caps_unref (caps);
  }

  g_print ("Image Capture Resolution = %d x %d\n",
      app->capres.image_cap_width, app->capres.image_cap_height);
  return TRUE;
}

gboolean
set_video_resolution (int new_res)
{
  GstCaps *caps = NULL;
  gint width = 0, height = 0;
  if (new_res == app->capres.vid_res_index) {
    g_print ("\nAlready on same video capture resolution\n");
    return TRUE;
  }
  if (!get_video_capture_resolution (new_res))
    return FALSE;

  //configure video
  g_object_get (app->ele.svc_vidvconv_out_filter, "caps", &caps, NULL);
  caps = gst_caps_make_writable (caps);

  gst_caps_set_simple (caps, "width", G_TYPE_INT,
      app->capres.video_cap_width, "height", G_TYPE_INT,
      app->capres.video_cap_height, NULL);

  g_object_set (app->ele.svc_vidvconv_out_filter, "caps", caps, NULL);
  gst_caps_unref (caps);

  if (camera_need_reconfigure (new_res, CAPTURE_PAD_VIDEO)) {
    g_object_get (app->ele.cap_filter, "caps", &caps, NULL);
    caps = gst_caps_make_writable (caps);

    get_max_resolution (app->capres.current_max_res, &width, &height);
    gst_caps_set_simple (caps, "width", G_TYPE_INT,
        width, "height", G_TYPE_INT, height, NULL);

    g_object_set (app->ele.cap_filter, "caps", caps, NULL);
    gst_caps_unref (caps);
  }
  g_print ("Video Capture Resolution = %d x %d\n",
      app->capres.video_cap_width, app->capres.video_cap_height);
  return TRUE;
}

void
set_saturation (gfloat dval)
{
  app->saturation = dval;
  g_object_set (G_OBJECT (app->ele.vsrc), "saturation", dval, NULL);
}

void
set_exposure_saturation (gfloat dval)
{
  app->exposure_compensation = dval;
  g_object_set (G_OBJECT (app->ele.vsrc), "exposurecompensation", dval, NULL);
}

void
set_whitebalance (gint val)
{
  app->whitebalance = val;
  g_object_set (G_OBJECT (app->ele.vsrc), "wbmode", val, NULL);
}

void
set_timeout(gint val)
{
  app->timeout = val;
  g_object_set (G_OBJECT (app->ele.vsrc), "timeout", val, NULL);
}

static void set_flip (gint val)
{
  app->flip_method = val;
  g_object_set (G_OBJECT (app->ele.svc_imgvconv), "flip-method", val, NULL);
  g_object_set (G_OBJECT (app->ele.svc_vidvconv), "flip-method", val, NULL);
}

static void
set_encoder_profile (H264EncProfileType profile)
{
  const gchar * profile_name;
  guint profile_id;

  if (profile < PROFILE_BASELINE || profile > PROFILE_HIGH) {
    g_print("Invalid value for profile\n");
    return;
  }

  if (app->encset.video_enc != FORMAT_H264_HW) {
    g_print("Profile only supported for H.264 encoder\n");
    return;
  }

  if (app->mode == CAPTURE_VIDEO && recording) {
    g_print("Cannot set profile while recording video\n");
    return;
  }

  switch(profile) {
    case PROFILE_BASELINE:
      if(app->encset.hw_enc_type == HW_OMX_ENC)
        profile_id = 1;
      else
        profile_id = 0;
      profile_name = "Baseline";
      break;
    case PROFILE_MAIN:
      profile_id = 2;
      profile_name = "Main";
      break;
    case PROFILE_HIGH:
      if(app->encset.hw_enc_type == HW_OMX_ENC)
        profile_id = 8;
      else
        profile_id = 4;
      profile_name = "High";
      break;
  }

  if (app->ele.vid_enc) {
    g_object_set(G_OBJECT(app->ele.vid_enc), "profile", profile_id, NULL);
  }

  app->encset.video_enc_profile = profile;

  g_print("Encoder Profile = %s\n", profile_name);
}

/**
  * Set encode file name.
  *
  * @param muxer_type : container type
  */
static void
set_new_file_name (int muxer_type)
{
  gchar filename[100];
  gchar * file_ext = NULL;
  switch (app->mode) {
    case CAPTURE_VIDEO:
      switch (muxer_type) {
        case FILE_MP4:
          file_ext = "mp4";
          break;
        case FILE_3GP:
          file_ext = "3gp";
          break;
        case FILE_MKV:
          file_ext = "mkv";
          break;
        case FILE_H265:
          file_ext = "h265";
          break;
        default:
          file_ext = "mp4";
          break;
      }
      break;
    case CAPTURE_IMAGE:
      switch (muxer_type) {
        case FORMAT_JPEG_SW:
        case FORMAT_JPEG_HW:
        default:
          file_ext = "jpg";
          break;
      }
      break;
    default:
      g_print ("Invalid capture Mode, cannot set filename\n");
      break;
  }

  sprintf (filename, "%s_%ld_s%02d_%05d.%s", app->file_name, (long) getpid(),
    app->sensor_id, app->capture_count++, file_ext);

  CALL_GUI_FUNC(set_video_file_name, filename);

  gst_element_set_state (app->ele.video_sink, GST_STATE_NULL);
  g_object_set (G_OBJECT (app->ele.video_sink), "location", filename, NULL);
  gst_element_set_locked_state (app->ele.video_sink, FALSE);
  gst_element_set_state (app->ele.video_sink, GST_STATE_PLAYING);
}

/**
  * Create image encoder element.
  *
  * @param iencoder : image encoder type
  */
static gboolean
get_image_encoder (GstElement ** iencoder)
{
  switch (app->encset.image_enc) {
    case FORMAT_JPEG_SW:
      *iencoder = gst_element_factory_make (NVGST_SW_IMAGE_ENC, NULL);
      break;
    case FORMAT_JPEG_HW:
      *iencoder = gst_element_factory_make (NVGST_DEFAULT_IMAGE_ENC, NULL);
      break;
    default:
      *iencoder = gst_element_factory_make (NVGST_DEFAULT_IMAGE_ENC, NULL);
      break;
  }

  if (!(*iencoder)) {
    app->return_value = -1;
    NVGST_ERROR_MESSAGE ("Can't Create image encoder element\n");
    return FALSE;
  }

  return TRUE;
}

/**
  * Create video encoder element.
  *
  * @param vencoder : video encoder type
  */
static gboolean
get_video_encoder (GstElement ** vencoder)
{
  switch (app->encset.video_enc) {
    case FORMAT_H264_HW:
      if(app->encset.hw_enc_type == HW_OMX_ENC)
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_H264_VENC, NULL);
      else
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_H264_VENC, NULL);
      set_encoder_bitrate (app->encset.bitrate);
      set_encoder_profile (app->encset.video_enc_profile);
      break;
    case FORMAT_VP8_HW:
      if(app->encset.hw_enc_type == HW_OMX_ENC)
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_VP8_VENC, NULL);
      else
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_VP8_VENC, NULL);
      set_encoder_bitrate (app->encset.bitrate);
      break;
    case FORMAT_H265_HW:
      if(app->encset.hw_enc_type == HW_OMX_ENC)
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_H265_VENC, NULL);
      else
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_H265_VENC, NULL);
      set_encoder_bitrate (app->encset.bitrate);
      break;
    case FORMAT_VP9_HW:
      if(app->encset.hw_enc_type == HW_OMX_ENC)
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_VP9_VENC, NULL);
      else
        *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_VP9_VENC, NULL);
      set_encoder_bitrate (app->encset.bitrate);
      break;
    default:
      *vencoder = gst_element_factory_make (NVGST_PRIMARY_H264_VENC, NULL);
      break;
  }

  g_object_set (*vencoder, "control-rate", app->encset.controlrate, NULL);
  if (app->encset.enabletwopassCBR)
    g_object_set (*vencoder, "EnableTwopassCBR", app->encset.enabletwopassCBR, NULL);
  g_print ("Encoder control-rate = %u\n", app->encset.controlrate);
  g_print ("Encoder EnableTwopassCBR = %d\n", app->encset.enabletwopassCBR);

  if (!(*vencoder)) {
    app->return_value = -1;
    NVGST_ERROR_MESSAGE ("Can't Create video encoder element\n");
    return FALSE;
  }

  return TRUE;
}

// create parser based on encset.video_enc 
static gboolean get_parser (GstElement ** parser)
{
  switch (app->encset.video_enc) {
    case FORMAT_H264_HW:
      *parser = gst_element_factory_make (NVGST_PRIMARY_H264_PARSER, NULL);
      break;
    case FORMAT_H265_HW:
      *parser = gst_element_factory_make (NVGST_PRIMARY_H265_PARSER, NULL);
      break;
    default:
      *parser = gst_element_factory_make (NVGST_PRIMARY_IDENTITY, NULL);
      break;
  }
  return TRUE;
}


static gboolean get_muxer (GstElement ** muxer)
{
  if (app->encset.video_enc == FORMAT_VP9_HW) {
    if (app->file_type != FILE_MKV) {
        NVGST_WARNING_MESSAGE
            ("VP9 is only supported format with MKV in current GST version. "
            "Selecting MKV as container\n");
        app->file_type = FILE_MKV;
    }
  }

  app->muxer_is_identity = FALSE;

  switch (app->file_type) {
    case FILE_MP4:
      *muxer = gst_element_factory_make (NVGST_PRIMARY_MP4_MUXER, NULL);
      break;
    case FILE_3GP:
      *muxer = gst_element_factory_make (NVGST_PRIMARY_3GP_MUXER, NULL);
      break;
    case FILE_MKV:
      *muxer = gst_element_factory_make (NVGST_PRIMARY_MKV_MUXER, NULL);
      break;
    case FILE_H265:
      *muxer = gst_element_factory_make (NVGST_PRIMARY_IDENTITY, NULL);
      app->muxer_is_identity = TRUE;
      break;
    default:
      *muxer = gst_element_factory_make (NVGST_PRIMARY_MP4_MUXER, NULL);
      break;
  }

  if (!(*muxer)) {
    app->return_value = -1;
    NVGST_ERROR_MESSAGE ("Can't Create muxer element\n");
    return FALSE;
  }

  return TRUE;
}

// get bitrate based on resolution 
static guint get_encoder_bitrate ()
{
    guint bitrate
      if (app->capres.vid_res_index < VR_1280x720)
        bitrate = NVGST_DEFAULT_480P_ENCODER_BITRATE;
      else if (app->capres.vid_res_index >= VR_1280x720
          && app->capres.vid_res_index < VR_1920x1080)
        bitrate = NVGST_DEFAULT_720P_ENCODER_BITRATE;
      else if (app->capres.vid_res_index >= VR_1920x1080
          && app->capres.vid_res_index < VR_3840x2160)
        bitrate = NVGST_DEFAULT_1080P_ENCODER_BITRATE;
      else if (app->capres.vid_res_index >= VR_3840x2160)
        bitrate = NVGST_DEFAULT_2160P_ENCODER_BITRATE;
    }
    app->encset.bitrate = bitrate;
    g_print ("bitrate = %u\n", app->encset.bitrate);
    g_object_set (G_OBJECT (app->ele.vid_enc), "bitrate", app->encset.bitrate,
        NULL);
#ifdef WITH_STREAMING
    if (app->streaming_mode)
      g_object_set (G_OBJECT (app->ele.colorspace_conv), "bitrate",
          app->encset.bitrate, NULL);
#endif
  }
}

// initialize capture parameters 
static void init_capture_params()
{
    app->lock = malloc(sizeof(*(app->lock)));
    g_mutex_init (app->lock);
    app->cond = malloc(sizeof(*(app->cond)));
    g_cond_init (app->cond);

    app->file_type = NVGST_DEFAULT_FILE_TYPE;
    app->file_name = g_strdup (NVGST_DEFAULT_FILENAME);

    // default capture resolutions
    int video_capres = VR_1920x1080; 
    int image_capres = IR_1280x720; 
    app->capres.video_cap_width = video_capture_width[video_capres];
    app->capres.video_cap_height = video_capture_height[video_capres];
    app->capres.vid_res_index = video_capres;
    app->capres.image_cap_width = image_capture_width[image_capres];
    app->capres.image_cap_height = image_capture_height[image_capres];
    app->capres.img_res_index = image_capres;

    // default encoding settings 
    app->encset.image_enc = NVGST_DEFAULT_IMAGE_ENCODER;
    app->encset.video_enc = NVGST_DEFAULT_VIDEO_ENCODER;
    set_encoder_bitrate (NVGST_DEFAULT_480P_ENCODER_BITRATE);
    set_encoder_profile (NVGST_DEFAULT_VIDEO_ENCODER_PROFILE);
    app->encset.controlrate = NVGST_DEFAULT_VIDEO_ENCODER_CONTROLRATE;
    app->encset.enabletwopassCBR = NVGST_DEFAULT_VIDEO_ENCODER_TWOPASSCBR;

    for (int i = 0; i < app->num_cameras; i++) {
        CamSet& camset = app->camsets[i];
        camset.flip_method = NVGST_DEFAULT_FLIP_METHOD;
        camset->whitebalance = NVGST_DEFAULT_WHITEBALANCE;
        camset->saturation = NVGST_DEFAULT_SATURATION;
        camset->sensor_id = NVGST_DEFAULT_SENSOR_ID;
        camset->sensor_mode = NVGST_DEFAULT_SENSOR_MODE;
        camset->display_id = NVGST_DEFAULT_DISPLAY_ID;
        camset->exposure_timerange = NULL;
        camset->gain_range = NULL;
        camset->isp_digital_gainrange = NULL;
        camset->enableAeLock = FALSE;
        camset->enableAwbLock = FALSE;
        camset->exposure_compensation = NVGST_DEFAULT_EXPOSURE_COMPENSATION;
        camset->ae_antibanding = NVGST_DEFAULT_AEANTIBANDING;
        camset->tnr_mode = NVGST_DEFAULT_TNR_MODE;
        camset->ee_mode = NVGST_DEFAULT_EE_MODE;
        camset->ee_strength = NVGST_DEFAULT_EE_STRENGTH;
        camset->tnr_strength = NVGST_DEFAULT_TNR_STRENGTH;
    }
}

// check important parameters 
static gboolean check_capture_params()
{
  gboolean ret = TRUE;

  return ret;
}

// create csi camera capture bin 
static gboolean create_csi_cap_bin(gint index)
{
    GstPad *pad = NULL;
    GstCaps *caps = NULL;
    GstCapsFeatures *feature = NULL;
    gint width = 0, height = 0;

    assert(index < app->num_cameras); 
    CamSet& camset = app->camsets[index]; 
    CamBin& pipeline = app->pipeline.cams[index]; 

    pipeline.vsrc = gst_element_factory_make (NVGST_VIDEO_CAPTURE_SRC_CSI_ARGUS, NULL);
    if (!pipeline.vsrc) {
        NVGST_ERROR_MESSAGE_V ("Element camera capture creation failed (%d)", index);
        goto fail;
    }

    // CSI camera properties tuning 
    g_object_set (G_OBJECT (pipeline.vsrc), "wbmode", camset.whitebalance, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "timeout", camset.timeout, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "saturation", camset.saturation, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "sensor-id", camset.sensor_id, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "sensor-mode", camset.sensor_mode, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "aelock", camset.enableAeLock, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "awblock", camset.enableAwbLock, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "exposurecompensation", camset.exposure_compensation, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "aeantibanding", camset.ae_antibanding, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "tnr-mode", camset.tnr_mode , NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "ee-mode", camset.ee_mode , NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "tnr-strength", camset.tnr_strength, NULL);
    g_object_set (G_OBJECT (pipeline.vsrc), "ee-strength", camset.ee_strength, NULL);

    if (camset.exposure_timerange != NULL)
      g_object_set (G_OBJECT (pipeline.vsrc), "exposuretimerange", camset.exposure_timerange, NULL);

    if (camset.gain_range != NULL)
      g_object_set (G_OBJECT (pipeline.vsrc), "gainrange", camset.gain_range, NULL);

    if (camset.isp_digital_gainrange != NULL)
      g_object_set (G_OBJECT (pipeline.vsrc), "ispdigitalgainrange", camset.isp_digital_gainrange, NULL);

    // caps 
    pipeline.cap_filter = gst_element_factory_make (NVGST_DEFAULT_CAPTURE_FILTER, NULL);
    if (!pipeline.cap_filter) {
        NVGST_ERROR_MESSAGE_V ("Element cpature filter creation failed (%d)", index);
        goto fail;
    }

    app->capres.current_max_res = MAX (app->capres.vid_res_index,  app->capres.img_res_index);
    get_max_resolution (app->capres.current_max_res, &width, &height);
    caps = gst_caps_new_simple ("video/x-raw", "format", G_TYPE_STRING, "NV12",
            "width", G_TYPE_INT, width, "height", G_TYPE_INT, height, "framerate",
            GST_TYPE_FRACTION, NVGST_DEFAULT_CAPTURE_FPS, 1, NULL);

    feature = gst_caps_features_new ("memory:NVMM", NULL);
    gst_caps_set_features (caps, 0, feature);

    g_object_set (pipeline.cap_filter, "caps", caps, NULL);
    gst_caps_unref (caps);

    pipeline.cap_bin = gst_bin_new ("cap_bin");
    gst_bin_add_many (GST_BIN (pipeline.cap_bin), pipeline.vsrc, pipeline.cap_filter, NULL);
    if ((gst_element_link (pipeline.cap_bin), pipeline.vsrc, pipeline.cap_filter)) != TRUE) 
    {
        NVGST_ERROR_MESSAGE_V ("Link failed for elements camera capture & capture filter (%d)", index);
        goto fail;
    }

    pad = gst_element_get_static_pad (pipeline.cap_filter, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of capture filter (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.cap_bin, gst_ghost_pad_new ("src", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

static gboolean create_vid_conv_bin (gint index)
{
    GstPad *pad = NULL;
    GstCaps *caps = NULL;
    GstCapsFeatures *feature = NULL;

    assert(index < app->num_cameras); 
    CamBin& pipeline = app->pipeline.cams[index]; 

    pipeline.vconv = gst_element_factory_make (NVGST_DEFAULT_VIDEO_CONVERTER, NULL);
    if (!pipeline.vconv) {
        NVGST_ERROR_MESSAGE_V ("Element video convert creation failed (%d)", index);
        goto fail;
    }
    g_object_set (pipeline.vconv, "flip-method", app->flip_method, NULL);

    pipeline.vconv_out_filter = gst_element_factory_make (NVGST_DEFAULT_CAPTURE_FILTER, NULL);
    if (!pipeline.vconv_out_filter) {
        NVGST_ERROR_MESSAGE_V ("Element video convert filter creation failed (%d)", index);
        goto fail;
    }

    caps = gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width", G_TYPE_INT, app->capres.video_cap_width,
        "height", G_TYPE_INT, app->capres.video_cap_height, NULL);

    feature = gst_caps_features_new ("memory:NVMM", NULL);
    gst_caps_set_features (caps, 0, feature);

    g_object_set (pipeline.vconv_out_filter, "caps", caps, NULL);
    gst_caps_unref (caps);

    pipeline.vid_conv_bin = gst_bin_new ("vid_conv_bin");
    gst_bin_add_many (GST_BIN (pipeline.vid_conv_bin), pipeline.vconv, pipeline.vconv_out_filter, NULL);
    if (!gst_element_link_many (pipeline.vid_conv_bin), pipeline.vconv, pipeline.vconv_out_filter, NULL)) 
    {
        NVGST_ERROR_MESSAGE_V ("Link failed for elements video convert & convert filter (%d)", index);
        goto fail;
    }

    pad = gst_element_get_static_pad (pipeline.vconv, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static sink pad of video convert (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.vid_conv_bin, gst_ghost_pad_new ("sink", pad));
    gst_object_unref (GST_OBJECT (pad));


    pad = gst_element_get_static_pad (pipeline.vcon_out_filter, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of video convert filter (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.vid_conv_bin, gst_ghost_pad_new ("src", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

// video encoder bin 
// nvv4l2h264env -> parser 
static gboolean create_vid_bin(gint index)
{
    GstPad *pad = NULL;

    assert(index < app->num_cameras); 
    CamBin& pipeline = app->pipeline.cams[index]; 

    if (!get_video_encoder (&pipeline.venc)) {
        NVGST_ERROR_MESSAGE ("Video encoder element could not be created (%d)", index);
        goto fail;
    }

    if (!get_parser (&pipeline.parser)) {
        NVGST_ERROR_MESSAGE ("Video parser element could not be created (%d)", index);
        goto fail;
    }

    pipeline.vid_bin = gst_bin_new ("vid_bin");
    gst_bin_add_many (GST_BIN (pipeline.vid_bin), pipeline.venc, pipeline.parser, NULL);

    if ((gst_element_link (pipeline.venc, pipeline.parser)) != TRUE) {
        NVGST_ERROR_MESSAGE ("Elements could not link video encoder & parser (%d)", index);
        goto fail;
    }

    pad = gst_element_get_static_pad (pipeline.venc, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static sink pad of video encoder (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.venc_bin, gst_ghost_pad_new ("sink", pad));
    gst_object_unref (GST_OBJECT (pad));

    pad = gst_element_get_static_pad (pipeline.parser, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of parser (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.venc_bin, gst_ghost_pad_new ("src", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

// create file sink that support multiple video streams 
// qtmax -> filesink 
static gboolean create_file_bin()
{
    GstPad *pad = NULL;

    if (!get_muxer (&app->pipeline.muxer)) {
        NVGST_ERROR_MESSAGE ("Video muxer element could not be created.");
        goto fail;
    }

    app->pipeline.file_sink = gst_element_factory_make (NVGST_DEFAULT_FILE_SINK, NULL);
    if (!app->pipeline.file_sink) {
        NVGST_ERROR_MESSAGE ("File sink element could not be created.");
        goto fail;
    }
    g_object_set (G_OBJECT (app->pipeline.file_sink),
        "location", DEFAULT_FILE_LOCATION, "async", FALSE, "sync", FALSE, NULL);

    app->pipeline.file_bin = gst_bin_new ("file_bin");
    gst_bin_add_many (GST_BIN (app->pipeline.file_bin), 
        app->pipeline.muxer, app->pipeline.file_sink, NULL);

    if ((gst_element_link (app->pipeline.muxer, app->pipeline.file_sink)) != TRUE) {
        NVGST_ERROR_MESSAGE ("Elements could not link muxer & file_sink\n");
        goto fail;
    }

    pad = gst_element_get_request_pad (app->pipeline.muxer, "sink_%u");
    if (!pad) {
        NVGST_ERROR_MESSAGE ("can't get request sink pad of muxer");
        goto fail;
    }

    gst_element_add_pad (app->pipeline.sink_bin, gst_ghost_pad_new ("sink_%u", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

// image conv bin 
// nvvidconv -> cap_filter 
static gboolean create_img_conv_bin (gint index)
{
    GstPad *pad = NULL;
    GstCaps *caps = NULL;
    GstCapsFeatures *feature = NULL;

    assert(index < app->num_cameras); 
    CamBin& pipeline = app->pipeline.cams[index];
   
    pipeline.iconv = gst_element_factory_make (NVGST_DEFAULT_VIDEO_CONVERTER, NULL);
    if (!pipeline.iconv) {
        NVGST_ERROR_MESSAGE_V ("Element image convert creation failed (%d)", index);
        goto fail;
    }

    g_object_set (pipeline.iconv, "flip-method", app->flip_method, NULL);

    pipeline.iconv_out_filter = gst_element_factory_make (NVGST_DEFAULT_CAPTURE_FILTER, NULL);
    if (!pipeline.iconv_out_filter) {
        NVGST_ERROR_MESSAGE_V ("Element image convert filter creation failed (%d)", index);
        goto fail;
    }

    caps = gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "I420",
        "width", G_TYPE_INT, app->capres.image_cap_width,
        "height", G_TYPE_INT, app->capres.image_cap_height, NULL);

    feature = gst_caps_features_new ("memory:NVMM", NULL);
    gst_caps_set_features (caps, 0, feature);

    g_object_set (pipeline.iconv_out_filter, "caps", caps, NULL);
    gst_caps_unref (caps);

    pipeline.img_conv_bin = gst_bin_new ("img_conv_bin");
    gst_bin_add_many (GST_BIN (pipeline.img_conv_bin), pipeline.iconv, pipeline.iconv_out_filter, NULL);
    if (!gst_element_link_many (pipeline.img_conv_bin), pipeline.iconv, pipeline.iconv_out_filter, NULL)) 
    {
        NVGST_ERROR_MESSAGE_V ("Element link failed between image convert & convert filter (%d)", index);
        goto fail;
    }

    pad = gst_element_get_static_pad (pipeline.iconv, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE ("can't get static sink pad of image convert (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.img_conv_bin, gst_ghost_pad_new ("sink", pad));
    gst_object_unref (GST_OBJECT (pad));


    pad = gst_element_get_static_pad (pipeline.icon_out_filter, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE ("can't get static src pad of image convert filter (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.img_conv_bin, gst_ghost_pad_new ("src", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

/**
  * Write encoded image to file.
  *
  * @param fsink  : image sink
  * @param buffer : gst buffer
  * @param pad    : element pad
  * @param udata  : the gpointer to user data
  **/
static void 
cam_image_captured (GstElement * fsink, GstBuffer * buffer, GstPad * pad, gpointer udata)
{
  GstMapInfo info;
  if (gst_buffer_map (buffer, &info, GST_MAP_READ)) 
  {
    if (info.size > 0) {
      // FILE *fp = NULL;
      // gchar outfile[100];
      // gchar temp[100];
      // memset (outfile, 0, sizeof (outfile));
      // memset (temp, 0, sizeof (temp));

      // strncat (outfile, app->file_name, sizeof(outfile) - 1);
      // sprintf (outfile + strlen(outfile), "_%ld", (long) getpid());
      // sprintf (temp, "_s%02d_%05d.jpg", app->sensor_id, app->capture_count++);
      // strcat (outfile, temp);

      // fp = fopen (outfile, "wb");
      // if (fp == NULL) {
      //   g_print ("Can't open file for Image Capture!\n");
      //   app->cap_success = FALSE;
      // } 
      // else {
      //   if (info.size != fwrite (info.data, 1, info.size, fp)) {
      //     g_print ("Can't write data in file, No Space left on Device!\n");
      //     app->cap_success = FALSE;
      //     fclose (fp);
      //     if (remove (outfile) != 0)
      //       g_print ("Unable to delete the file\n");
      //   } 
      //   else {
      //     app->cap_success = TRUE;
      //     fclose (fp);
      //   }
      // }
      // app->capcount++;
      assert(app->stream_server); 
      int index = 0; 
      app->stream_server->update_image(info.data, info.size, index); 
      gst_buffer_unmap (buffer, &info);
    } 
    else {
      NVGST_WARNING_MESSAGE ("image buffer probe failed\n");
    }
  }
}

// create image bin 
// nvjpegenv -> fake_sink 
static gboolean create_img_bin (int index)
{
    GstPad *pad = NULL;
    assert(index < app->num_cameras); 
    CamBin& pipeline = app->pipeline.cams[index]; 

    if (!get_image_encoder (&pipeline.ienc)) {
        NVGST_ERROR_MESSAGE_V ("Image encoder element could not be created (%d)", index);
        goto fail;
    }

    pipeline.fake_sink = gst_element_factory_make (NVGST_DEFAULT_IENC_SINK, NULL);
    if (!pipeline.fake_sink) {
        NVGST_ERROR_MESSAGE_V ("Image fake sink element could be created (%d)", index);
        goto fail;
    }
    g_object_set (G_OBJECT (pipeline.fake_sink), "signal-handoffs", TRUE, NULL);
    g_signal_connect (G_OBJECT (pipeline.fake_sink), "handoff", G_CALLBACK (cam_image_captured), index);

    pipleline.img_bin = gst_bin_new ("img_bin");
    gst_bin_add_many (GST_BIN (pipleline.img_bin), pipleline.ienc, pipleline.fake_sink, NULL);

    if ((gst_element_link (pipleline.ienc, pipleline.fake_sink)) != TRUE) {
        NVGST_ERROR_MESSAGE_V ("Elements could not link image encoder & fake_sink (%d)", index);
        goto fail;
    }

    pad = gst_element_get_static_pad (pipeline.ienc, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static sink pad of image encoder (%d)", index);
        goto fail;
    }
    gst_element_add_pad (pipeline.img_bin, gst_ghost_pad_new ("sink", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

/**
  * handler on the bus
  *
  * @param bus  : a GstBus
  * @param msg  : the GstMessage
  * @param data : user data that has been given
  */
static GstBusSyncReply bus_sync_handler (GstBus * bus, GstMessage * msg, gpointer data)
{
    switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_ELEMENT:
        if (GST_MESSAGE_SRC (msg) == GST_OBJECT (app->pipeline.pipeline)) {
            const GstStructure *structure;
            structure = gst_message_get_structure (msg);
            if (gst_structure_has_name (structure, "video-done")) {
            NVGST_INFO_MESSAGE ("video-capture-done");
            } 
            else if (gst_structure_has_name (structure, "GstBinForwarded")) {
            GstMessage *child_msg;
            if (gst_structure_has_field (structure, "message")) {
                const GValue *val = gst_structure_get_value (structure, "message");
                if (G_VALUE_TYPE (val) == GST_TYPE_MESSAGE) {
                child_msg = (GstMessage *) g_value_get_boxed (val);
                if (GST_MESSAGE_TYPE (child_msg) == GST_MESSAGE_EOS &&
                    GST_MESSAGE_SRC (child_msg) == GST_OBJECT (app->pipeline.vid_bin))
                {
                    if (app->reset_thread)
                    g_thread_unref (app->reset_thread);
                    app->reset_thread = g_thread_new (NULL, reset_elements, NULL);
                }
                }
            }
            }
        }
        return GST_BUS_PASS;

        default:
        return GST_BUS_PASS;
    }
}

/**
  * Handle received message
  *
  * @param bus  : a GstBus
  * @param msg  : the GstMessage
  * @param data : user data that has been given
  */
static gboolean bus_callback (GstBus * bus, GstMessage * msg, gpointer data)
{
    switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_ERROR:
        {
            GError *err = NULL;
            gchar *name, *debug = NULL;
            name = gst_object_get_path_string (msg->src);
            gst_message_parse_error (msg, &err, &debug);
            g_printerr ("ERROR on bus: by %s: %s\n", name, err->message);
            if (debug != NULL)
                g_printerr ("debug info:\n%s\n", debug);
            g_error_free (err);
            g_free (debug);
            g_free (name);
            app->return_value = -1;
            g_main_loop_quit (loop);
            break;
        }
        case GST_MESSAGE_STATE_CHANGED:
        {
            GstState old, new_state, pending;
            gst_message_parse_state_changed (msg, &old, &new_state, &pending);
            GST_DEBUG_OBJECT (GST_OBJECT (msg->src),
                "changed state from %s to %s, pending %s\n",
                gst_element_state_get_name (old), gst_element_state_get_name (new_state),
                gst_element_state_get_name (pending));

            if (GST_MESSAGE_SRC (msg) == GST_OBJECT (app->pipeline.pipeline)
                && pending == GST_STATE_VOID_PENDING && old == GST_STATE_PAUSED
                && new_state == GST_STATE_PLAYING) {
            }
            break;
        }
        case GST_MESSAGE_EOS:
        {
            // restart_capture_pipeline ();
            g_print("EOS\n"); 
            break;
        }
        case GST_MESSAGE_APPLICATION:
        {
            const GstStructure *s;
            s = gst_message_get_structure (msg);

            if (gst_structure_has_name (s, "NvGstAppInterrupt")) {
                g_print ("Terminating the capture pipeline ...\n");
                g_main_loop_quit (loop);
            }
            break;
        }
        case GST_MESSAGE_ELEMENT:
            break;

        default:
            break;
    }
    return TRUE;
}

// nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,framerate=(fraction)30/1" ! 
// nvvidconv flip-method=0 left=0 top=0 right=1920 bottom=1080 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,pixel-aspect-ratio=1/1" ! 
// queue ! nvv4l2h264enc maxperf-enable=true bitrate=8000000 ! h264parse ! queue ! muxer.video_0 

// nvarguscamerasrc sensor-id=1 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,framerate=(fraction)30/1" ! 
// nvvidconv flip-method=0 left=0 top=0 right=1920 bottom=1080 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,pixel-aspect-ratio=1/1" ! 
// queue ! nvv4l2h264enc maxperf-enable=true bitrate=8000000 ! h264parse !  queue ! muxer.video_1 

// qtmux name=muxer ! filesink location="test_pair.mp4image" sync=false async=false

static gboolean create_csi_capture_pipeline()
{
    GstBus *bus = NULL;
    GstPad *srcpad = NULL;
    GstPad *sinkpad = NULL;
    gchar sink_x[64];

    // Create the pipeline 
    app->pipeline.pipeline = gst_pipeline_new ("csi_capture_pipeline");;
    if (!app->pipeline.pipeline) {
        NVGST_ERROR_MESSAGE ("CSI capture pipeline creation failed");
        goto fail;
    }

    bus = gst_pipeline_get_bus (GST_PIPELINE (app->pipeline.pipeline));
    gst_bus_set_sync_handler (bus, bus_sync_handler, app->pipeline.pipeline, NULL);
    gst_bus_add_watch (bus, bus_callback, NULL);
    gst_object_unref (bus);

    g_object_set (app->pipeline.pipeline, "message-forward", TRUE, NULL);

    // Create the video file sink bin 
    if (!create_file_sink_bin(index)) {
        NVGST_ERROR_MESSAGE_V ("video file sink bin creation failed");
        goto fail;
    }

    // Create two cameras bins 
    for (int index = 0; index < app->num_cameras; index++) { 
        CamBin& pipeline = app->pipeline->cams[index]; 

        // Create csi capture chain elements 
        if (!create_csi_cap_bin(index)) {
            NVGST_ERROR_MESSAGE_V ("cap bin %d creation failed", index);
            goto fail;
        }

        // Create video encode chain elements 
        if (!create_vid_enc_bin(index)) {
            NVGST_ERROR_MESSAGE_V ("video encode bin %d creation failed", index);
            goto fail;
        }

        // Create image encode chain elements 
        if (!create_img_enc_bin(index)) {
            NVGST_ERROR_MESSAGE_V ("image encode bin %d creation failed", index);
            goto fail;
        }

        // Create video scaling elements 
        if (!create_vid_conv_bin(index)) {
            NVGST_ERROR_MESSAGE_V ("video conv bin % creation failed", index);
            goto fail;
        }

       // Create image scaling elements 
        if (!create_img_conv_bin(index)) {
            NVGST_ERROR_MESSAGE_V ("image conv bin %d creation failed", index);
            goto fail;
        }

        // Create capture tee for capture streams 
        pipeline.cap_tee = gst_element_factory_make (NVGST_PRIMARY_STREAM_SELECTOR, NULL);
        if (!pipeline.cap_tee) {
            NVGST_ERROR_MESSAGE_V ("capture tee %d creation failed", index);
            goto fail;
        }

        g_object_set (G_OBJECT (pipeline.cap_tee), "name", "cam_t", NULL);
        g_object_set (G_OBJECT (pipeline.cap_tee), "mode", GST_NVCAM_MODE_VIDEO, NULL);

        // Create encode queues 
        pipeline.venc_q = gst_element_factory_make (NVGST_PRIMARY_QUEUE, NULL);
        pipeline.ienc_q = gst_element_factory_make (NVGST_PRIMARY_QUEUE, NULL);
        if (!pipeline.venc_q || !pipeline.ienc_q) {
            NVGST_ERROR_MESSAGE_V ("encode queue %d creation failed", index);
            goto fail;
        }

        // Add elements to camera pipeline 
        gst_bin_add_many (GST_BIN (pipeline), pipeline.cap_bin, pipeline.cap_tee, 
                pipeline.venc_q, pipeline.vid_conv_bin, pipeline.vid_bin, 
                pipeline.ienc_q, pipeline.img_conv_bin, pipeline.img_bin, 
                NULL);

        // Manually link the Tee with video queue 
        srcpad = gst_element_get_static_pad (pipeline.cap_tee, "vid_src");
        sinkpad = gst_element_get_static_pad (pipeline.venc_q, "sink");
        if (!sinkpad || !srcpad) {
            NVGST_ERROR_MESSAGE ("fail to get pads from tee & venc_q\n");
            goto fail;
        }
        if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
            NVGST_ERROR_MESSAGE ("fail to link tee & venc_q\n");
            goto fail;
        }

        // probe on venc pipeline 
        app->venc_probe_id = gst_pad_add_probe (sinkpad, GST_PAD_PROBE_TYPE_BUFFER, venc_buf_prob, index, NULL);
        gst_object_unref (sinkpad);
        gst_object_unref (srcpad);

        // Manually link the video queue with video scaling 
        srcpad = gst_element_get_static_pad (pipeline.venc_q, "src");
        sinkpad = gst_element_get_static_pad (pipeline.vid_conv_bin, "sink");
        if (!sinkpad || !srcpad) {
            NVGST_ERROR_MESSAGE ("fail to get pads from video queue & video conv\n");
            goto fail;
        }
        if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
            NVGST_ERROR_MESSAGE ("fail to link video queue & video conv\n");
            goto fail;
        }
        gst_object_unref (sinkpad);
        gst_object_unref (srcpad);

        // Manually link the Tee with image queue 
        srcpad = gst_element_get_static_pad (pipeline.cap_tee, "pre_src");
        sinkpad = gst_element_get_static_pad (pipeline.ienc_q, "sink");
        if (!sinkpad || !srcpad) {
            NVGST_ERROR_MESSAGE ("fail to get pads from tee & ienc_q\n");
            goto fail;
        }
        if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
            NVGST_ERROR_MESSAGE ("fail to link tee & ienc_q\n");
            goto fail;
        }
        
        // probe on ienc pipeline 
        app->ienc_probe_id = gst_pad_add_probe (sinkpad, GST_PAD_PROBE_TYPE_BUFFER, ienc_buf_prob, index, NULL);
        gst_object_unref (sinkpad);
        gst_object_unref (srcpad);

        // Manually link the image queue with image scaling 
        srcpad = gst_element_get_static_pad (pipeline.ienc_q, "src");
        sinkpad = gst_element_get_static_pad (pipeline.img_conv_bin, "sink");
        if (!sinkpad || !srcpad) {
            NVGST_ERROR_MESSAGE ("fail to get pads from image queue & image conv\n");
            goto fail;
        }
        if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
            NVGST_ERROR_MESSAGE ("fail to link image queue & image conv\n");
            goto fail;
        }
        gst_object_unref (sinkpad);
        gst_object_unref (srcpad);

        // link the capture bin with Tee 
        if (!gst_element_link (pipeline.cap_bin, pipeline.cap_tee) {
            NVGST_ERROR_MESSAGE ("fail to link capture bin & tee\n");
            goto fail;
        }

        // link the video scaling bin with encode bin 
        if (!gst_element_link (pipeline.vid_conv_bin, pipeline.vid_bin)) {
            NVGST_ERROR_MESSAGE ("fail to link vid_conv_bin & vid_bin\n");
            goto fail;
        }

        // link the image scaling bin with encode bin 
        if (!gst_element_link (pipeline.img_conv_bin, pipeline.img_bin)) {
            NVGST_ERROR_MESSAGE ("fail to link img_conv_bin & img_bin");
            goto fail;
        }

        // Manually link the video encode bin to video sindk 
        g_snprintf (sink_x, sizeof (sink_x), "sink_%d", index);
        srcpad = gst_element_get_static_pad (pipeline.vid_bin, "src");
        sinkpad = gst_element_get_static_pad (pipeline.file_bin, sink_x);
        if (!sinkpad || !srcpad) {
            NVGST_ERROR_MESSAGE_V ("fail to get pads from venc & file sink %d", index);
            goto fail;
        }
        if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
            NVGST_ERROR_MESSAGE_V ("fail to link venc & file sink %d", index);
            goto fail;
        }
        gst_object_unref (sinkpad);
        gst_object_unref (srcpad);
    }

    return TRUE;

fail:
  app->return_value = -1;
  return FALSE;
}

// create capture pipeline 
// destroy_pipeline + create_pipeline should restart 
// the pipeline with updated parameters 
gboolean create_pipeline()
{
  gboolean ret = TRUE;

  // Check for capture parameters 
  if (!check_capture_params ()) {
    NVGST_ERROR_MESSAGE ("Invalid capture parameters");
    goto fail;
  }

  // Create pipeline for csi cameras 
  ret = create_csi_capture_pipeline ();
  if (!ret) {
    NVGST_ERROR_MESSAGE ("can't create capture pipeline\n");
    goto fail;
  }

  // Capture pipeline created, now start capture 
  GST_INFO_OBJECT (app->ele.camera, "camera ready");

  if (GST_STATE_CHANGE_FAILURE == gst_element_set_state (app->ele.camera, GST_STATE_PLAYING)) {
    NVGST_CRITICAL_MESSAGE ("can't set camera to playing\n");
    goto fail;
  }

  /* Dump Capture - Playing Pipeline into the dot file
   * Set environment variable "export GST_DEBUG_DUMP_DOT_DIR=/tmp"
   * Run nvgstcapture and 0.00.00.*-nvgstcapture-playing.dot
   * file will be generated.
   * Run "dot -Tpng 0.00.00.*-nvgstcapture-playing.dot > image.png"
   * image.png will display the running capture pipeline.
   */
  GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS (GST_BIN(app->ele.camera), GST_DEBUG_GRAPH_SHOW_ALL, "nvgstcapture-playing");

  return ret;

fail:
  app->return_value = -1;
  return FALSE;
}

// Destroy pipeline.
void destroy_pipeline()
{
  GstPad *sinkpad = NULL;

  // stop the pipeline
  if (!app->pipeline.pipeline) return;
  if (GST_STATE_CHANGE_FAILURE == gst_element_set_state (app->pipeline.pipeline, GST_STATE_NULL)) {
    g_warning ("can't set camera pipeline to null\n");
  }

  // remove enc probe 
  sinkpad = gst_element_get_static_pad (app->pipeline.venc_q, "sink");
  gst_pad_remove_probe (sinkpad, app->venc_probe_id);
  gst_object_unref (sinkpad);

  sinkpad = gst_element_get_static_pad (app->pipeline.ienc_q, "sink");
  gst_pad_remove_probe (sinkpad, app->ienc_probe_id);
  gst_object_unref (sinkpad);

  if (app->reset_thread) {
    g_thread_unref (app->reset_thread);
    app->reset_thread = NULL;
  }

  gst_object_unref (GST_OBJECT (app->pipeline.pipeline));
  app->pipeline.camera = NULL;
  app->pipeline.vsrc = NULL;
  app->pipeline.vsink = NULL;
  app->pipeline.cap_filter = NULL;
  app->pipeline.cap_tee = NULL;
  app->pipeline.prev_q = NULL;
  app->pipeline.venc_q = NULL;
  app->pipeline.ienc_q = NULL;
  app->pipeline.img_enc = NULL;
  app->pipeline.vid_enc = NULL;
  app->pipeline.muxer = NULL;
  app->pipeline.img_sink = NULL;
  app->pipeline.video_sink = NULL;

  app->pipeline.capbin = NULL;
  app->pipeline.vid_bin = NULL;
  app->pipeline.img_bin = NULL;
  app->pipeline.svsbin = NULL;

  app->pipeline.vid_enc_conv = NULL;
  app->pipeline.vid_enc_cap_filter = NULL;
}

// restart capture  
void restart_pipeline (void)
{
  destroy_pipeline ();
  g_usleep (250000);
  if (!create_pipeline ()) {
    app->return_value = -1;
    g_main_loop_quit (loop);
  }
}

// exit capture 
gboolean exit_capture (gpointer data)
{
  g_main_loop_quit (loop);
  return FALSE;
}

bool set_capture_params()
{

}

bool start_recording()
{
    set_new_file_name();
    for (int i = 0; i < app_num_cams; i++) {
        CamBin& pipeline = app->pipeline->cams[i]; 
        g_signal_emit_by_name (G_OBJECT (pipeline.cap_tee), "start-capture");
    }
    return true; 
}

bool stop_recording()
{
    for (int i = 0; i < app->num_cams; i++) {
        CamBin& pipeline = app->pipeline->cams[i]; 
        g_signal_emit_by_name(G_OBJECT(pipeline.cap_tee), "stop-capture");
        gst_pad_send_event(gst_element_get_static_pad(pipeline.venc_q, "sink"), gst_event_new_eos());
    }
    return true; 
}

int main (int argc, char *argv[])
{
    app = &appctx;
    memset (app, 0, sizeof(AppCtx));
    capture_init_params(); 

    _intr_setup ();
    g_timeout_add (400, check_for_interrupt, NULL);

    gst_init (&argc, &argv);
    loop = g_main_loop_new (NULL, FALSE);
    g_main_loop_run (loop);

    // create the mjpeg server 

    // create the capture pipeline 
    if (create_pipeline ()) {
        NVGST_INFO_MESSAGE ("Loop run");
        g_main_loop_run (loop);
    } 
    else
        NVGST_CRITICAL_MESSAGE ("Pipeline creation failed");

    destroy_pipeline ();
    NVGST_INFO_MESSAGE ("Capture completed");

    if (loop)
        g_main_loop_unref (loop);

    if (app->lock) {
        g_mutex_clear (app->lock);
        app->lock = NULL;
    }

    if (app->cond) {
        g_cond_clear (app->cond);
        app->cond = NULL;
    }

    g_free (app->file_name);
    g_free (app->lock);
    g_free (app->cond);

    NVGST_INFO_MESSAGE ("Camera capture will now exit");
    return ((app->return_value == -1) ? -1 : 0);
}
