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

static AppCtx appctx;
AppCtx *app = NULL;

static GMainLoop *loop = NULL;
static gboolean cintr = FALSE;

static void capture_init_params();
static gboolean create_capture_pipeline();
static void destroy_capture_pipeline();


static gboolean check_capture_params (void);
static gboolean create_native_capture_pipeline (void);

static bool set_encoder_bitrate (guint bitrate);
static bool set_encoder_profile (H264EncProfileType profile);
void set_capture_device_node (void);
static void print_help (void);
static void set_new_file_name();
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
ienc_buf_prob (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
static GstPadProbeReturn
venc_buf_prob (GstPad * pad, GstPadProbeInfo * info, gpointer u_data);
gboolean get_preview_resolution (gint res);
static gboolean get_image_capture_resolution (gint res);
static gboolean get_video_capture_resolution (gint res);
static gboolean camera_need_reconfigure (int new_res,
    CapturePadType current_pad);


gboolean recording = FALSE;
static gboolean snapshot = FALSE;

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

#define WIDTH_RES   640, 1280, 1920, 3264, 3264, 2560, 2592, 4032
#define HEIGHT_RES  480, 720,  1080, 1848, 2464, 1440, 1944, 3040

gint prevres_width[] = { WIDTH_RES };
gint prevres_height[] = { HEIGHT_RES };
static gint image_capture_width[] = { WIDTH_RES };
static gint image_capture_height[] = { HEIGHT_RES };
static gint video_capture_width[] = { WIDTH_RES };
static gint video_capture_height[] = { HEIGHT_RES };


// initialize capture parameters 
static void capture_init_params()
{
    app->num_cams = MAX_NUM_CAMS; 
    // app->lock = malloc(sizeof(*(app->lock)));
    // g_mutex_init (app->lock);
    // app->cond = malloc(sizeof(*(app->cond)));
    // g_cond_init (app->cond);

    // app->file_type = NVGST_DEFAULT_FILE_TYPE;
    // app->file_name = g_strdup (NVGST_DEFAULT_FILENAME);

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
    // set_encoder_bitrate (NVGST_DEFAULT_480P_ENCODER_BITRATE);
    // set_encoder_profile (NVGST_DEFAULT_VIDEO_ENCODER_PROFILE);
    app->encset.controlrate = NVGST_DEFAULT_VIDEO_ENCODER_CONTROLRATE;
    app->encset.enabletwopassCBR = NVGST_DEFAULT_VIDEO_ENCODER_TWOPASSCBR;

    for (int i = 0; i < app->num_cams; i++) {
        CamSet& camset = app->camsets[i];
        camset.cam_index = i; 
        camset.sensor_id = i; 
        camset.flip_method = NVGST_DEFAULT_FLIP_METHOD;
        // camset->whitebalance = NVGST_DEFAULT_WHITEBALANCE;
        // camset->saturation = NVGST_DEFAULT_SATURATION;
        // camset->sensor_mode = NVGST_DEFAULT_SENSOR_MODE;
        // camset->display_id = NVGST_DEFAULT_DISPLAY_ID;
        // camset->exposure_timerange = NULL;
        // camset->gain_range = NULL;
        // camset->isp_digital_gainrange = NULL;
        // camset->enableAeLock = FALSE;
        // camset->enableAwbLock = FALSE;
        // camset->exposure_compensation = NVGST_DEFAULT_EXPOSURE_COMPENSATION;
        // camset->ae_antibanding = NVGST_DEFAULT_AEANTIBANDING;
        // camset->tnr_mode = NVGST_DEFAULT_TNR_MODE;
        // camset->ee_mode = NVGST_DEFAULT_EE_MODE;
        // camset->ee_strength = NVGST_DEFAULT_EE_STRENGTH;
        // camset->tnr_strength = NVGST_DEFAULT_TNR_STRENGTH;
    }
}


static gboolean check_for_interrupt (gpointer data)
{
  if (cintr) {
    cintr = FALSE;

    gst_element_post_message (GST_ELEMENT (app->gst.pipeline),
        gst_message_new_application (GST_OBJECT (app->gst.pipeline),
            gst_structure_new ("NvGstAppInterrupt",
                "message", G_TYPE_STRING, "Pipeline interrupted", NULL)));

    return FALSE;
  }
  return TRUE;
}

static void _intr_handler (int signum)
{
    struct sigaction action;
    NVGST_INFO_MESSAGE ("User Interrupted");
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

static GstPadProbeReturn venc_buf_prob (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{

  return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn ienc_buf_prob (GstPad * pad, GstPadProbeInfo * info, gpointer u_data)
{

  return GST_PAD_PROBE_OK;
}

// /**
//   * get the max capture resolutions
//   *
//   * @param res : resolution index
//   */
// static void
// get_max_resolution (gint res, gint * width, gint * height)
// {
//   if (app->use_cus_res) {
//     *width = app->capres.cus_prev_width;
//     *height = app->capres.cus_prev_height;
//   } else {
//     *width = image_capture_width[res];
//     *height = image_capture_height[res];
//   }
// }

// static gboolean
// get_image_capture_resolution (gint res)
// {
//   gboolean ret = TRUE;

//   if ( (app->cam_src == NV_CAM_SRC_CSI) ||
//       (app->cam_src == NV_CAM_SRC_EGLSTREAM) ) {
//     if ((res < IR_1280x720) || (res > IR_4032x3040)) {
//       g_print ("Invalid image capture resolution\n");
//       return FALSE;
//     }
//   } else {
//     if ((res < IR_640x480) || (res > IR_1920x1080)) {
//       g_print ("Invalid image capture resolution\n");
//       return FALSE;
//     }
//   }
//   app->capres.image_cap_width = image_capture_width[res];
//   app->capres.image_cap_height = image_capture_height[res];
//   app->capres.img_res_index = res;

//   return ret;
// }

// static gboolean
// get_video_capture_resolution (gint res)
// {
//   gboolean ret = TRUE;

//   if ( (app->cam_src == NV_CAM_SRC_CSI) ||
//       (app->cam_src == NV_CAM_SRC_EGLSTREAM) ){
//     if ((res < VR_1280x720) || (res > VR_4032x3040)) {
//       g_print ("Invalid video capture resolution\n");
//       return FALSE;
//     }
//   } else {
//     if ((res < VR_640x480) || (res > VR_1280x720)) {
//       g_print ("Invalid video capture resolution\n");
//       return FALSE;
//     }
//   }
//   app->capres.video_cap_width = video_capture_width[res];
//   app->capres.video_cap_height = video_capture_height[res];
//   app->capres.vid_res_index = res;

//   return ret;
// }

// static gpointer
// reset_elements (gpointer data)
// {
//   gst_element_set_state (app->ele.venc_q, GST_STATE_READY);
//   gst_element_set_state (app->ele.vid_bin, GST_STATE_READY);
//   gst_element_set_state (app->ele.svc_vidbin, GST_STATE_READY);

//   gst_element_sync_state_with_parent (app->ele.venc_q);
//   gst_element_sync_state_with_parent (app->ele.vid_bin);
//   gst_element_sync_state_with_parent (app->ele.svc_vidbin);

//   return NULL;
// }


// void
// set_mode (gint newMode)
// {
//   if (newMode != 1 && newMode != 2) {
//     newMode = NVGST_DEFAULT_CAPTURE_MODE;
//     g_print ("Invalid input mode, setting mode to image-capture = 1 \n");
//   }
//   g_print ("Changing capture mode to %d\n", newMode);
//   g_print ("(1): image\n(2): video\n");

//   if (app->cam_src == NV_CAM_SRC_CSI) {
//     g_object_set (app->ele.cap_tee, "mode", newMode, NULL);
//   } else {
//     destroy_capture_pipeline ();
//     g_usleep (250000);
//     app->mode = newMode;
//     if (!create_capture_pipeline ()) {
//       app->return_value = -1;
//       g_main_loop_quit (loop);
//     }
//   }
//   app->mode = newMode;
// }

// gboolean
// set_preview_resolution (int new_res)
// {
//   GstCaps *caps = NULL;
//   gint width = 0, height = 0;
//   if (new_res == app->capres.prev_res_index) {
//     g_print ("\nAlready on same preview resolution\n");
//     return TRUE;
//   }
//   if (!get_preview_resolution (new_res))
//     return FALSE;

//   g_object_get (app->ele.svc_prevconv_out_filter, "caps", &caps, NULL);
//   caps = gst_caps_make_writable (caps);

//   gst_caps_set_simple (caps, "width", G_TYPE_INT,
//       app->capres.preview_width, "height", G_TYPE_INT,
//       app->capres.preview_height, NULL);

//   g_object_set (app->ele.svc_prevconv_out_filter, "caps", caps, NULL);
//   gst_caps_unref (caps);

//   if (camera_need_reconfigure (new_res, CAPTURE_PAD_PREV)) {
//     g_object_get (app->ele.cap_filter, "caps", &caps, NULL);
//     caps = gst_caps_make_writable (caps);

//     get_max_resolution (app->capres.current_max_res, &width, &height);
//     gst_caps_set_simple (caps, "width", G_TYPE_INT,
//         width, "height", G_TYPE_INT, height, NULL);

//     g_object_set (app->ele.cap_filter, "caps", caps, NULL);
//     gst_caps_unref (caps);
//   }

// #if !GUI
// {
//   GstElement *vsink = app->ele.vsink;

//   if (vsink && GST_IS_VIDEO_OVERLAY (vsink)) {
//     if (app->capres.preview_width < app->disp.display_width
//         || app->capres.preview_height < app->disp.display_height) {
//       app->disp.width = app->capres.preview_width;
//       app->disp.height = app->capres.preview_height;
//     } else {
//       app->disp.width = app->disp.display_width;
//       app->disp.height = app->disp.display_height;
//     }
//     g_mutex_lock (app->lock);

//     if (app->disp.window)
//       nvgst_destroy_window (&app->disp);
//     nvgst_create_window (&app->disp, "nvgstcapture");
//     gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (vsink),
//         (gulong) app->disp.window);
//     gst_video_overlay_expose (GST_VIDEO_OVERLAY (vsink));

//     g_mutex_unlock (app->lock);
//  }
// }
// #endif
//   g_print ("Preview resolution = %d x %d\n",
//       app->capres.preview_width, app->capres.preview_height);

//   return TRUE;
// }

// gboolean
// set_image_resolution (int new_res)
// {
//   GstCaps *caps = NULL;
//   gint width = 0, height = 0;
//   if (new_res == app->capres.img_res_index) {
//     g_print ("\nAlready on same image capture resolution\n");
//     return TRUE;
//   }
//   if (!get_image_capture_resolution (new_res))
//     return FALSE;

//   //configure image
//   g_object_get (app->ele.svc_imgvconv_out_filter, "caps", &caps, NULL);
//   caps = gst_caps_make_writable (caps);

//   gst_caps_set_simple (caps, "width", G_TYPE_INT,
//       app->capres.image_cap_width, "height", G_TYPE_INT,
//       app->capres.image_cap_height, NULL);

//   g_object_set (app->ele.svc_imgvconv_out_filter, "caps", caps, NULL);
//   gst_caps_unref (caps);

//   if (camera_need_reconfigure (new_res, CAPTURE_PAD_IMAGE)) {
//     g_object_get (app->ele.cap_filter, "caps", &caps, NULL);
//     caps = gst_caps_make_writable (caps);

//     get_max_resolution (app->capres.current_max_res, &width, &height);
//     gst_caps_set_simple (caps, "width", G_TYPE_INT,
//         width, "height", G_TYPE_INT, height, NULL);

//     g_object_set (app->ele.cap_filter, "caps", caps, NULL);
//     gst_caps_unref (caps);
//   }

//   g_print ("Image Capture Resolution = %d x %d\n",
//       app->capres.image_cap_width, app->capres.image_cap_height);
//   return TRUE;
// }

// gboolean
// set_video_resolution (int new_res)
// {
//   GstCaps *caps = NULL;
//   gint width = 0, height = 0;
//   if (new_res == app->capres.vid_res_index) {
//     g_print ("\nAlready on same video capture resolution\n");
//     return TRUE;
//   }
//   if (!get_video_capture_resolution (new_res))
//     return FALSE;

//   //configure video
//   g_object_get (app->ele.svc_vidvconv_out_filter, "caps", &caps, NULL);
//   caps = gst_caps_make_writable (caps);

//   gst_caps_set_simple (caps, "width", G_TYPE_INT,
//       app->capres.video_cap_width, "height", G_TYPE_INT,
//       app->capres.video_cap_height, NULL);

//   g_object_set (app->ele.svc_vidvconv_out_filter, "caps", caps, NULL);
//   gst_caps_unref (caps);

//   if (camera_need_reconfigure (new_res, CAPTURE_PAD_VIDEO)) {
//     g_object_get (app->ele.cap_filter, "caps", &caps, NULL);
//     caps = gst_caps_make_writable (caps);

//     get_max_resolution (app->capres.current_max_res, &width, &height);
//     gst_caps_set_simple (caps, "width", G_TYPE_INT,
//         width, "height", G_TYPE_INT, height, NULL);

//     g_object_set (app->ele.cap_filter, "caps", caps, NULL);
//     gst_caps_unref (caps);
//   }
//   g_print ("Video Capture Resolution = %d x %d\n",
//       app->capres.video_cap_width, app->capres.video_cap_height);
//   return TRUE;
// }

// void
// set_saturation (gfloat dval)
// {
//   app->saturation = dval;
//   g_object_set (G_OBJECT (app->ele.vsrc), "saturation", dval, NULL);
// }

// void
// set_exposure_saturation (gfloat dval)
// {
//   app->exposure_compensation = dval;
//   g_object_set (G_OBJECT (app->ele.vsrc), "exposurecompensation", dval, NULL);
// }

// void
// set_whitebalance (gint val)
// {
//   app->whitebalance = val;
//   g_object_set (G_OBJECT (app->ele.vsrc), "wbmode", val, NULL);
// }

// void
// set_timeout(gint val)
// {
//   app->timeout = val;
//   g_object_set (G_OBJECT (app->ele.vsrc), "timeout", val, NULL);
// }

// static void set_flip (gint val)
// {
//   app->flip_method = val;
//   g_object_set (G_OBJECT (app->ele.svc_imgvconv), "flip-method", val, NULL);
//   g_object_set (G_OBJECT (app->ele.svc_vidvconv), "flip-method", val, NULL);
// }

static bool set_encoder_bitrate (guint bitrate)
{
    if (bitrate == 0) 
    { 
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

    for (int i = 0; i < app->num_cams; i++) {
        if (app->gst.cams[i].venc)
            g_object_set(G_OBJECT(app->gst.cams[i].venc), "bitrate", bitrate, NULL);
    }

    app->encset.bitrate = bitrate;
    NVGST_INFO_MESSAGE_V("bitrate = %u", app->encset.bitrate);
    return true; 
}

static bool set_encoder_profile (H264EncProfileType profile)
{
    const gchar * profile_name;
    guint profile_id;

    if (profile < PROFILE_BASELINE || profile > PROFILE_HIGH) {
        NVGST_ERROR_MESSAGE("Invalid value for profile");
        return false;
    }

    if (app->encset.video_enc != FORMAT_H264_HW) {
        NVGST_ERROR_MESSAGE("Profile only supported for H.264 encoder");
        return false;
    }

    switch(profile) {
        case PROFILE_BASELINE:
            profile_id = 0;
            profile_name = "Baseline";
            break;
        case PROFILE_MAIN:
            profile_id = 2;
            profile_name = "Main";
            break;
        case PROFILE_HIGH:
            profile_id = 4;
            profile_name = "High";
        break;
    }

    for (int i = 0; i < app->num_cams; i++) {
        if (app->gst.cams[i].venc) 
            g_object_set(G_OBJECT(app->gst.cams[i].venc), "profile", profile_id, NULL);
    }

    app->encset.video_enc_profile = profile;
    NVGST_INFO_MESSAGE_V("Encoder Profile = %s", profile_name);
    return true; 
}

static void set_new_file_name ()
{
    gchar filename[100];
    gchar * file_ext = "mp4";
    // switch (muxer_type) {
    //     case FILE_MP4:
    //         file_ext = "mp4";
    //         break;
    //     case FILE_3GP:
    //         file_ext = "3gp";
    //         break;
    //     case FILE_MKV:
    //         file_ext = "mkv";
    //         break;
    //     case FILE_H265:
    //         file_ext = "h265";
    //         break;
    //     default:
    //         file_ext = "mp4";
    //         break;
    // }

//   sprintf (filename, "%s_%ld_s%02d_%05d.%s", app->file_name, (long) getpid(),
//     app->sensor_id, app->capture_count++, file_ext);

//   gst_element_set_state (app->ele.video_sink, GST_STATE_NULL);
//   g_object_set (G_OBJECT (app->ele.video_sink), "location", filename, NULL);
//   gst_element_set_locked_state (app->ele.video_sink, FALSE);
//   gst_element_set_state (app->ele.video_sink, GST_STATE_PLAYING);
}


static gboolean get_image_encoder (GstElement ** iencoder)
{
    switch (app->encset.image_enc) {
        case FORMAT_JPEG_SW:
            NVGST_INFO_MESSAGE("create software JPEG encoder"); 
            *iencoder = gst_element_factory_make (NVGST_SW_IMAGE_ENC, NULL);
            break;
        case FORMAT_JPEG_HW:
            NVGST_INFO_MESSAGE("create hardware JPEG encoder"); 
            *iencoder = gst_element_factory_make (NVGST_DEFAULT_IMAGE_ENC, NULL);
            break;
        default:
            NVGST_INFO_MESSAGE("create hardware JPEG encoder as default"); 
            *iencoder = gst_element_factory_make (NVGST_DEFAULT_IMAGE_ENC, NULL);
            break;
    }

    if (!(*iencoder)) {
        app->return_value = -1;
        NVGST_ERROR_MESSAGE ("Can't create image encoder element");
        return FALSE;
    }

    return TRUE;
}


static gboolean get_video_encoder (GstElement ** vencoder)
{
    switch (app->encset.video_enc) 
    {
        case FORMAT_H264_HW:
            *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_H264_VENC, NULL);
            set_encoder_bitrate (app->encset.bitrate);
            set_encoder_profile (app->encset.video_enc_profile);
            break;
        case FORMAT_VP8_HW:
            *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_VP8_VENC, NULL);
            set_encoder_bitrate (app->encset.bitrate);
            break;
        case FORMAT_H265_HW:
            *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_H265_VENC, NULL);
            set_encoder_bitrate (app->encset.bitrate);
            break;
        case FORMAT_VP9_HW:
            *vencoder = gst_element_factory_make (NVGST_PRIMARY_V4L2_VP9_VENC, NULL);
            set_encoder_bitrate (app->encset.bitrate);
            break;
        default:
            *vencoder = gst_element_factory_make (NVGST_PRIMARY_H264_VENC, NULL);
            break;
    }

    if (!(*vencoder)) {
        app->return_value = -1;
        NVGST_ERROR_MESSAGE ("Can't Create video encoder element");
        return FALSE;
    }

    g_object_set (*vencoder, "control-rate", app->encset.controlrate, NULL);
    if (app->encset.enabletwopassCBR)
        g_object_set (*vencoder, "EnableTwopassCBR", app->encset.enabletwopassCBR, NULL);

    return TRUE;
}

// create parser based on encset.video_enc 
static gboolean get_parser (GstElement ** parser)
{
    switch (app->encset.video_enc) {
        case FORMAT_H264_HW:
            NVGST_INFO_MESSAGE("create parser for H264 encoder"); 
            *parser = gst_element_factory_make (NVGST_PRIMARY_H264_PARSER, NULL);
            break;
        case FORMAT_H265_HW:
            NVGST_INFO_MESSAGE("create parser for H265 encoder"); 
            *parser = gst_element_factory_make (NVGST_PRIMARY_H265_PARSER, NULL);
            break;
        default:
            NVGST_INFO_MESSAGE("create identity parser as default"); 
            *parser = gst_element_factory_make (NVGST_PRIMARY_IDENTITY, NULL);
            break;
    }
    return TRUE;
}


static gboolean get_muxer (GstElement ** muxer)
{
    if (app->encset.video_enc == FORMAT_VP9_HW) {
        NVGST_INFO_MESSAGE("create mux for VP9 encoder"); 
        if (app->file_type != FILE_MKV) {
            NVGST_WARNING_MESSAGE
                ("VP9 is only supported format with MKV in current GST version. "
                "Set MKV as container\n");
            app->file_type = FILE_MKV;
        }
    }

    app->muxer_is_identity = FALSE;
    switch (app->file_type) {
        case FILE_MP4:
            NVGST_INFO_MESSAGE("create mux for MP4 file"); 
            *muxer = gst_element_factory_make (NVGST_PRIMARY_MP4_MUXER, NULL);
            break;
        case FILE_3GP:
            NVGST_INFO_MESSAGE("create mux for 3GP file"); 
            *muxer = gst_element_factory_make (NVGST_PRIMARY_3GP_MUXER, NULL);
            break;
        case FILE_MKV:
            NVGST_INFO_MESSAGE("create mux for MKV file"); 
            *muxer = gst_element_factory_make (NVGST_PRIMARY_MKV_MUXER, NULL);
            break;
        case FILE_H265:
            NVGST_INFO_MESSAGE("create mux for H265 file"); 
            *muxer = gst_element_factory_make (NVGST_PRIMARY_IDENTITY, NULL);
            app->muxer_is_identity = TRUE;
            break;
        default:
            NVGST_INFO_MESSAGE("create mux for MP4 file as default"); 
            *muxer = gst_element_factory_make (NVGST_PRIMARY_MP4_MUXER, NULL);
            break;
    }

    if (!(*muxer)) {
        app->return_value = -1;
        NVGST_ERROR_MESSAGE ("Can't Create muxer element");
        return FALSE;
    }

    return TRUE;
}

// // get bitrate based on resolution 
// static guint get_encoder_bitrate ()
// {
//     guint bitrate
//       if (app->capres.vid_res_index < VR_1280x720)
//         bitrate = NVGST_DEFAULT_480P_ENCODER_BITRATE;
//       else if (app->capres.vid_res_index >= VR_1280x720
//           && app->capres.vid_res_index < VR_1920x1080)
//         bitrate = NVGST_DEFAULT_720P_ENCODER_BITRATE;
//       else if (app->capres.vid_res_index >= VR_1920x1080
//           && app->capres.vid_res_index < VR_3840x2160)
//         bitrate = NVGST_DEFAULT_1080P_ENCODER_BITRATE;
//       else if (app->capres.vid_res_index >= VR_3840x2160)
//         bitrate = NVGST_DEFAULT_2160P_ENCODER_BITRATE;
//     }
//     app->encset.bitrate = bitrate;
//     g_print ("bitrate = %u\n", app->encset.bitrate);
//     g_object_set (G_OBJECT (app->ele.vid_enc), "bitrate", app->encset.bitrate,
//         NULL);
// #ifdef WITH_STREAMING
//     if (app->streaming_mode)
//       g_object_set (G_OBJECT (app->ele.colorspace_conv), "bitrate",
//           app->encset.bitrate, NULL);
// #endif
//   }
// }



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

    assert(index < app->num_cams); 
    CamSet& camset = app->camsets[index]; 
    CamPipe& campipe = app->gst.cams[index]; 

    NVGST_INFO_MESSAGE_V("create capture %d", index); 
    campipe.vsrc = gst_element_factory_make (NVGST_VIDEO_CAPTURE_SRC_CSI_ARGUS, NULL);
    if (!campipe.vsrc) {
        NVGST_ERROR_MESSAGE_V ("Element capture creation failed (%d)", index);
        goto fail;
    }

    // CSI camera properties tuning 
    NVGST_INFO_MESSAGE_V("set capture params for camera %d", index); 
    g_object_set (G_OBJECT (campipe.vsrc), "sensor-id", camset.sensor_id, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "wbmode", camset.whitebalance, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "timeout", camset.timeout, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "saturation", camset.saturation, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "aelock", camset.enableAeLock, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "awblock", camset.enableAwbLock, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "exposurecompensation", camset.exposure_compensation, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "aeantibanding", camset.ae_antibanding, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "tnr-mode", camset.tnr_mode , NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "ee-mode", camset.ee_mode , NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "tnr-strength", camset.tnr_strength, NULL);
    // g_object_set (G_OBJECT (campipe.vsrc), "ee-strength", camset.ee_strength, NULL);

    // if (camset.exposure_timerange != NULL)
    //   g_object_set (G_OBJECT (campipe.vsrc), "exposuretimerange", camset.exposure_timerange, NULL);

    // if (camset.gain_range != NULL)
    //   g_object_set (G_OBJECT (campipe.vsrc), "gainrange", camset.gain_range, NULL);

    // if (camset.isp_digital_gainrange != NULL)
    //   g_object_set (G_OBJECT (campipe.vsrc), "ispdigitalgainrange", camset.isp_digital_gainrange, NULL);

    // caps 
    NVGST_INFO_MESSAGE_V("create capture caps filter for camera %d", index); 
    campipe.cap_filter = gst_element_factory_make (NVGST_DEFAULT_CAPTURE_FILTER, NULL);
    if (!campipe.cap_filter) {
        NVGST_ERROR_MESSAGE_V ("Element cpature filter creation failed (%d)", index);
        goto fail;
    }

    width = app->capres.video_cap_width; 
    height = app->capres.video_cap_height; 
    caps = gst_caps_new_simple ("video/x-raw", "format", G_TYPE_STRING, "NV12",
            "width", G_TYPE_INT, width, "height", G_TYPE_INT, height, "framerate",
            GST_TYPE_FRACTION, NVGST_DEFAULT_CAPTURE_FPS, 1, NULL);

    feature = gst_caps_features_new ("memory:NVMM", NULL);
    gst_caps_set_features (caps, 0, feature);

    NVGST_INFO_MESSAGE_V("set capture caps for camera %d", index); 
    NVGST_INFO_MESSAGE_V("%s", gst_caps_to_string(caps)); 
    g_object_set (campipe.cap_filter, "caps", caps, NULL);
    gst_caps_unref (caps);

    NVGST_INFO_MESSAGE_V("create capture bin for camera %d", index); 
    campipe.cap_bin = gst_bin_new ("cap_bin");
    gst_bin_add_many (GST_BIN (campipe.cap_bin), campipe.vsrc, campipe.cap_filter, NULL);

    NVGST_INFO_MESSAGE_V("link capture and caps filter for camera %d", index); 
    if (!gst_element_link (campipe.vsrc, campipe.cap_filter)) {
        NVGST_ERROR_MESSAGE_V ("Link failed for elements camera capture & capture filter (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create ghost pad for camera %d", index); 
    pad = gst_element_get_static_pad (campipe.cap_filter, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of capture filter (%d)", index);
        goto fail;
    }
    gst_element_add_pad (campipe.cap_bin, gst_ghost_pad_new ("src", pad));
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

    assert(index < app->num_cams); 
    CamPipe& campipe = app->gst.cams[index]; 

    NVGST_INFO_MESSAGE_V("create video convert for camera %d", index); 
    campipe.vconv = gst_element_factory_make (NVGST_DEFAULT_VIDEO_CONVERTER_CSI, NULL);
    if (!campipe.vconv) {
        NVGST_ERROR_MESSAGE_V ("Element video convert creation failed (%d)", index);
        goto fail;
    }
    g_object_set (campipe.vconv, "flip-method", app->camsets[index].flip_method, NULL);

    NVGST_INFO_MESSAGE_V("create video convert caps filter for camera %d", index); 
    campipe.vconv_out_filter = gst_element_factory_make (NVGST_DEFAULT_CAPTURE_FILTER, NULL);
    if (!campipe.vconv_out_filter) {
        NVGST_ERROR_MESSAGE_V ("Element video convert filter creation failed (%d)", index);
        goto fail;
    }

    caps = gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width", G_TYPE_INT, app->capres.video_cap_width,
        "height", G_TYPE_INT, app->capres.video_cap_height, NULL);

    feature = gst_caps_features_new ("memory:NVMM", NULL);
    gst_caps_set_features (caps, 0, feature);

    NVGST_INFO_MESSAGE_V("set video convert caps filter for camera %d", index); 
    NVGST_INFO_MESSAGE_V("%s", gst_caps_to_string(caps)); 
    g_object_set (campipe.vconv_out_filter, "caps", caps, NULL);
    gst_caps_unref (caps);

    NVGST_INFO_MESSAGE_V("create video convert bin for camera %d", index);
    campipe.vid_conv_bin = gst_bin_new ("vid_conv_bin");
    gst_bin_add_many (GST_BIN (campipe.vid_conv_bin), campipe.vconv, campipe.vconv_out_filter, NULL);

    NVGST_INFO_MESSAGE_V("link video convert to caps filter for camera %d", index);
    if (!gst_element_link_many (campipe.vconv, campipe.vconv_out_filter, NULL)) {
        NVGST_ERROR_MESSAGE_V ("Link failed for elements video convert & convert filter (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create ghost sink pad for video convert bin for camera %d", index);
    pad = gst_element_get_static_pad (campipe.vconv, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static sink pad of video convert (%d)", index);
        goto fail;
    }
    gst_element_add_pad (campipe.vid_conv_bin, gst_ghost_pad_new ("sink", pad));
    gst_object_unref (GST_OBJECT (pad));

    NVGST_INFO_MESSAGE_V("create ghost src pad for video convert bin for camera %d", index);
    pad = gst_element_get_static_pad (campipe.vconv_out_filter, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of video convert filter (%d)", index);
        goto fail;
    }
    gst_element_add_pad (campipe.vid_conv_bin, gst_ghost_pad_new ("src", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

// video encoder bin 
// nvv4l2h264env -> parser 
static gboolean create_vid_enc_bin(gint index)
{
    GstPad *pad = NULL;

    assert(index < app->num_cams); 
    CamPipe& campipe = app->gst.cams[index]; 

    NVGST_INFO_MESSAGE_V("create video encode for camera %d", index); 
    if (!get_video_encoder (&campipe.venc)) {
        NVGST_ERROR_MESSAGE_V ("Video encoder element could not be created (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create video encode parser for camera %d", index);
    if (!get_parser (&campipe.parser)) {
        NVGST_ERROR_MESSAGE_V ("Video parser element could not be created (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create video encode bin for camera %d", index);
    campipe.vid_enc_bin = gst_bin_new ("vid_enc_bin");
    gst_bin_add_many (GST_BIN (campipe.vid_enc_bin), campipe.venc, campipe.parser, NULL);

    NVGST_INFO_MESSAGE_V("link video encode and parser for camera %d", index);
    if ((gst_element_link (campipe.venc, campipe.parser)) != TRUE) {
        NVGST_ERROR_MESSAGE_V ("Elements could not link video encoder & parser (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create ghost sink pad for video encode bin for camera %d", index);
    pad = gst_element_get_static_pad (campipe.venc, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static sink pad of video encoder (%d)", index);
        goto fail;
    }
    gst_element_add_pad (campipe.vid_enc_bin, gst_ghost_pad_new ("sink", pad));
    gst_object_unref (GST_OBJECT (pad));

    NVGST_INFO_MESSAGE_V("create ghost src pad for video encode bin for camera %d", index);
    pad = gst_element_get_static_pad (campipe.parser, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of parser (%d)", index);
        goto fail;
    }
    gst_element_add_pad (campipe.vid_enc_bin, gst_ghost_pad_new ("src", pad));
    gst_object_unref (GST_OBJECT (pad));

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

// create file sink that support multiple video streams 
// qtmax -> filesink 
static gboolean create_vid_sink_bin()
{
    GstPad *pad = NULL;
    gchar sink_x[64]; 

    NVGST_INFO_MESSAGE("create muxer"); 
    if (!get_muxer (&app->gst.muxer)) {
        NVGST_ERROR_MESSAGE ("Video muxer element could not be created.");
        goto fail;
    }

    NVGST_INFO_MESSAGE("create file sink"); 
    app->gst.file_sink = gst_element_factory_make (NVGST_DEFAULT_FILE_SINK, NULL);
    if (!app->gst.file_sink) {
        NVGST_ERROR_MESSAGE ("File sink element could not be created.");
        goto fail;
    }
    NVGST_INFO_MESSAGE_V("set file location to: %s", DEFAULT_FILE_LOCATION); 
    g_object_set (G_OBJECT (app->gst.file_sink),
        "location", DEFAULT_FILE_LOCATION, "async", FALSE, "sync", FALSE, NULL);

    NVGST_INFO_MESSAGE("create video sink bin"); 
    app->gst.vid_sink_bin = gst_bin_new ("vid_sink_bin");
    gst_bin_add_many (GST_BIN (app->gst.vid_sink_bin), 
        app->gst.muxer, app->gst.file_sink, NULL);

    NVGST_INFO_MESSAGE("link muxer to file sink"); 
    if ((gst_element_link (app->gst.muxer, app->gst.file_sink)) != TRUE) {
        NVGST_ERROR_MESSAGE ("Elements could not link muxer & file_sink\n");
        goto fail;
    }

    
    for (int index = 0; index < app->num_cams; index++) {
        g_snprintf (sink_x, sizeof (sink_x), "video_%d", index);
        NVGST_INFO_MESSAGE_V("create ghost %s pad for video sink bin", sink_x); 
        pad = gst_element_get_request_pad (app->gst.muxer, sink_x);
        if (!pad) {
            NVGST_ERROR_MESSAGE_V ("can't get request %s pad of muxer", sink_x);
            goto fail;
        }
        gst_element_add_pad (app->gst.vid_sink_bin, gst_ghost_pad_new (sink_x, pad));
        gst_object_unref (GST_OBJECT (pad));
    }

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

    assert(index < app->num_cams); 
    CamPipe& campipe = app->gst.cams[index];
   
    NVGST_INFO_MESSAGE_V("create image convert %d", index); 
    campipe.iconv = gst_element_factory_make (NVGST_DEFAULT_VIDEO_CONVERTER_CSI, NULL);
    if (!campipe.iconv) {
        NVGST_ERROR_MESSAGE_V ("Element image convert creation failed (%d)", index);
        goto fail;
    }
    g_object_set (campipe.iconv, "flip-method", app->camsets[index].flip_method, NULL);

    NVGST_INFO_MESSAGE_V("create image convert caps filter %d", index); 
    campipe.iconv_out_filter = gst_element_factory_make (NVGST_DEFAULT_CAPTURE_FILTER, NULL);
    if (!campipe.iconv_out_filter) {
        NVGST_ERROR_MESSAGE_V ("Element image convert filter creation failed (%d)", index);
        goto fail;
    }

    caps = gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "I420",
        "width", G_TYPE_INT, app->capres.image_cap_width,
        "height", G_TYPE_INT, app->capres.image_cap_height, NULL);

    feature = gst_caps_features_new ("memory:NVMM", NULL);
    gst_caps_set_features (caps, 0, feature);

    NVGST_INFO_MESSAGE_V("set image convert caps filter %d", index); 
    NVGST_INFO_MESSAGE_V("%s", gst_caps_to_string(caps)); 
    g_object_set (campipe.iconv_out_filter, "caps", caps, NULL);
    gst_caps_unref (caps);

    NVGST_INFO_MESSAGE_V("create image convert bin %d", index); 
    campipe.img_conv_bin = gst_bin_new ("img_conv_bin");
    gst_bin_add_many (GST_BIN (campipe.img_conv_bin), campipe.iconv, campipe.iconv_out_filter, NULL);

    NVGST_INFO_MESSAGE_V("link image convert & caps filter  %d", index); 
    if (!gst_element_link_many (campipe.iconv, campipe.iconv_out_filter, NULL)) {
        NVGST_ERROR_MESSAGE_V ("Element link failed between image convert & convert filter (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create ghost sink pad for image convert %d", index); 
    pad = gst_element_get_static_pad (campipe.iconv, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static sink pad of image convert %d", index);
        goto fail;
    }
    gst_element_add_pad (campipe.img_conv_bin, gst_ghost_pad_new ("sink", pad));
    gst_object_unref (GST_OBJECT (pad));

    NVGST_INFO_MESSAGE_V("create ghost src pad for image convert %d", index); 
    pad = gst_element_get_static_pad (campipe.iconv_out_filter, "src");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of image convert filter %d", index);
        goto fail;
    }
    gst_element_add_pad (campipe.img_conv_bin, gst_ghost_pad_new ("src", pad));
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
static void cam_image_captured (GstElement * fsink, GstBuffer * buffer, GstPad * pad, gpointer udata)
{
    int index = *(guint*)udata; 
    NVGST_INFO_MESSAGE_V("image captured: %d", index); 

    GstMapInfo info;
    if (!gst_buffer_map (buffer, &info, GST_MAP_READ)) {
        NVGST_WARNING_MESSAGE("failed map buffer"); 
        return; 
    }

    assert(app->stream_server); 
    app->stream_server->update_image(info.data, info.size, index); 
           
    gst_buffer_unmap (buffer, &info);
}

// create image bin 
// nvjpegenv -> fake_sink 
static gboolean create_img_enc_bin (int index)
{
    GstPad *pad = NULL;
    assert(index < app->num_cams); 
    CamPipe& campipe = app->gst.cams[index]; 

    NVGST_INFO_MESSAGE_V("create image encoder for camera %d", index); 
    if (!get_image_encoder (&campipe.ienc)) {
        NVGST_ERROR_MESSAGE_V ("Image encoder element could not be created (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create image sink for camera %d", index); 
    campipe.fake_sink = gst_element_factory_make (NVGST_DEFAULT_IENC_SINK, NULL);
    if (!campipe.fake_sink) {
        NVGST_ERROR_MESSAGE_V ("Image fake sink element could be created (%d)", index);
        goto fail;
    }
    g_object_set (G_OBJECT (campipe.fake_sink), "signal-handoffs", TRUE, NULL);
    g_signal_connect (G_OBJECT (campipe.fake_sink), "handoff", 
        G_CALLBACK (cam_image_captured), (void*)&app->camsets[index].cam_index);

    NVGST_INFO_MESSAGE_V("create image encode bin for camera %d", index); 
    campipe.img_enc_bin = gst_bin_new ("img_enc_bin");
    gst_bin_add_many (GST_BIN (campipe.img_enc_bin), campipe.ienc, campipe.fake_sink, NULL);

    NVGST_INFO_MESSAGE_V("link image encoder and fake sink for camera %d", index); 
    if ((gst_element_link (campipe.ienc, campipe.fake_sink)) != TRUE) {
        NVGST_ERROR_MESSAGE_V ("Elements could not link image encoder & fake_sink (%d)", index);
        goto fail;
    }

    NVGST_INFO_MESSAGE_V("create ghost sink for image encode bin for camera %d", index); 
    pad = gst_element_get_static_pad (campipe.ienc, "sink");
    if (!pad) {
        NVGST_ERROR_MESSAGE_V ("can't get static sink pad of image encoder (%d)", index);
        goto fail;
    }
    gst_element_add_pad (campipe.img_enc_bin, gst_ghost_pad_new ("sink", pad));
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
    switch (GST_MESSAGE_TYPE (msg)) 
    {
        case GST_MESSAGE_ELEMENT:
            if (GST_MESSAGE_SRC (msg) == GST_OBJECT (app->gst.pipeline)) {
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
                                GST_MESSAGE_SRC (child_msg) == GST_OBJECT (app->gst.vid_sink_bin))
                            {
                                // if (app->reset_thread)
                                // g_thread_unref (app->reset_thread);
                                // app->reset_thread = g_thread_new (NULL, reset_elements, NULL);
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

            if (GST_MESSAGE_SRC (msg) == GST_OBJECT (app->gst.pipeline)
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
                g_print ("Terminating the capture pipeline\n");
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

static gboolean create_csi_cam_bin(int index)
{
    GstPad *srcpad = NULL;
    GstPad *sinkpad = NULL;
    gchar name_x[64];

    assert(index < app->num_cams); 
    CamPipe& campipe = app->gst.cams[index]; 

    // Create csi capture chain elements 
    NVGST_INFO_MESSAGE_V("create csi capture bin %d", index);
    if (!create_csi_cap_bin(index)) {
        NVGST_ERROR_MESSAGE_V ("cap bin %d creation failed", index);
        goto fail;
    }

    // Create video encode chain elements 
    NVGST_INFO_MESSAGE_V("create video encode bin %d", index);
    if (!create_vid_enc_bin(index)) {
        NVGST_ERROR_MESSAGE_V ("video encode bin %d creation failed", index);
        goto fail;
    }

    // Create image encode chain elements 
    NVGST_INFO_MESSAGE_V("create image encode bin %d", index);
    if (!create_img_enc_bin(index)) {
        NVGST_ERROR_MESSAGE_V ("image encode bin %d creation failed", index);
        goto fail;
    }

    // Create video scaling elements 
    NVGST_INFO_MESSAGE_V("create video convert bin %d", index);
    if (!create_vid_conv_bin(index)) {
        NVGST_ERROR_MESSAGE_V ("video conv bin %d creation failed", index);
        goto fail;
    }

    // Create image scaling elements 
    NVGST_INFO_MESSAGE_V("create image convert bin %d", index);
    if (!create_img_conv_bin(index)) {
        NVGST_ERROR_MESSAGE_V ("image conv bin %d creation failed", index);
        goto fail;
    }

    // Create capture tee for capture streams 
    NVGST_INFO_MESSAGE_V("create capture tee %d", index);
    campipe.cap_tee = gst_element_factory_make (NVGST_PRIMARY_STREAM_SELECTOR, NULL);
    if (!campipe.cap_tee) {
        NVGST_ERROR_MESSAGE_V ("capture tee %d creation failed", index);
        goto fail;
    }

    g_object_set (G_OBJECT (campipe.cap_tee), "name", "cam_t", NULL);
    g_object_set (G_OBJECT (campipe.cap_tee), "mode", 2, NULL);

    // Create encode queues 
    NVGST_INFO_MESSAGE_V("create encode queues for camera %d", index);
    campipe.venc_q = gst_element_factory_make (NVGST_PRIMARY_QUEUE, NULL);
    campipe.ienc_q = gst_element_factory_make (NVGST_PRIMARY_QUEUE, NULL);
    if (!campipe.venc_q || !campipe.ienc_q) {
        NVGST_ERROR_MESSAGE_V ("encode queues %d creation failed", index);
        goto fail;
    }

    // Add elements to camera pipeline 
    g_snprintf(name_x, sizeof (name_x), "cam_bin_%d", index);
    campipe.cam_bin = gst_bin_new (name_x);
    gst_bin_add_many (GST_BIN (campipe.cam_bin), campipe.cap_bin, campipe.cap_tee, 
            campipe.venc_q, campipe.vid_conv_bin, campipe.vid_enc_bin, 
            campipe.ienc_q, campipe.img_conv_bin, campipe.img_enc_bin, 
            NULL);

    // Manually link the Tee with video queue 
    NVGST_INFO_MESSAGE_V("link tee & video encode queue %d", index);
    srcpad = gst_element_get_static_pad (campipe.cap_tee, "vid_src");
    sinkpad = gst_element_get_static_pad (campipe.venc_q, "sink");
    if (!sinkpad || !srcpad) {
        NVGST_ERROR_MESSAGE ("fail to get pads from tee & venc_q");
        goto fail;
    }
    if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
        NVGST_ERROR_MESSAGE ("fail to link tee & venc_q");
        goto fail;
    }

    // probe on venc pipeline 
    NVGST_INFO_MESSAGE_V("set video encode probe %d", index);
    app->venc_probe_id = gst_pad_add_probe (sinkpad, GST_PAD_PROBE_TYPE_BUFFER, 
                            venc_buf_prob, (void*)&app->camsets[index].cam_index, NULL);
    gst_object_unref (sinkpad);
    gst_object_unref (srcpad);

    // Manually link the video queue with video scaling 
    NVGST_INFO_MESSAGE_V("link video encode queue & video convert %d", index);
    srcpad = gst_element_get_static_pad (campipe.venc_q, "src");
    sinkpad = gst_element_get_static_pad (campipe.vid_conv_bin, "sink");
    if (!sinkpad || !srcpad) {
        NVGST_ERROR_MESSAGE ("fail to get pads from video queue & video conv");
        goto fail;
    }
    if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
        NVGST_ERROR_MESSAGE ("fail to link video queue & video conv");
        goto fail;
    }
    gst_object_unref (sinkpad);
    gst_object_unref (srcpad);

    // Manually link the Tee with image queue 
    NVGST_INFO_MESSAGE_V("link tee & image encode queue %d", index);
    srcpad = gst_element_get_static_pad (campipe.cap_tee, "pre_src");
    sinkpad = gst_element_get_static_pad (campipe.ienc_q, "sink");
    if (!sinkpad || !srcpad) {
        NVGST_ERROR_MESSAGE ("fail to get pads from tee & ienc_q");
        goto fail;
    }
    if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
        NVGST_ERROR_MESSAGE ("fail to link tee & ienc_q");
        goto fail;
    }
    
    // probe on ienc pipeline 
    NVGST_INFO_MESSAGE_V("set image encode probe %d", index);
    app->ienc_probe_id = gst_pad_add_probe (sinkpad, GST_PAD_PROBE_TYPE_BUFFER, 
                            ienc_buf_prob, (void*)&app->camsets[index].cam_index, NULL);
    gst_object_unref (sinkpad);
    gst_object_unref (srcpad);

    // Manually link the image queue with image scaling 
    NVGST_INFO_MESSAGE_V("link image encode queue & image convert %d", index);
    srcpad = gst_element_get_static_pad (campipe.ienc_q, "src");
    sinkpad = gst_element_get_static_pad (campipe.img_conv_bin, "sink");
    if (!sinkpad || !srcpad) {
        NVGST_ERROR_MESSAGE ("fail to get pads from image queue & image conv");
        goto fail;
    }
    if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
        NVGST_ERROR_MESSAGE ("fail to link image queue & image conv");
        goto fail;
    }
    gst_object_unref (sinkpad);
    gst_object_unref (srcpad);

    // link the capture bin with Tee 
    NVGST_INFO_MESSAGE_V("link capture bin & tee %d", index);
    if (!gst_element_link (campipe.cap_bin, campipe.cap_tee)) {
        NVGST_ERROR_MESSAGE ("fail to link capture bin & tee");
        goto fail;
    }

    // link the video scaling bin with encode bin 
    NVGST_INFO_MESSAGE_V("link video convert bin & video encode bin %d", index);
    if (!gst_element_link (campipe.vid_conv_bin, campipe.vid_enc_bin)) {
        NVGST_ERROR_MESSAGE ("fail to link vid_conv_bin & vid_enc_bin");
        goto fail;
    }

    // link the image scaling bin with encode bin 
    NVGST_INFO_MESSAGE_V("link image convert bin & image encode bin %d", index);
    if (!gst_element_link (campipe.img_conv_bin, campipe.img_enc_bin)) {
        NVGST_ERROR_MESSAGE ("fail to link img_conv_bin & img_enc_bin");
        goto fail;
    }
    
    // create ghost src pad for camera bin 
    NVGST_INFO_MESSAGE_V("create ghost src pad for camera bin %d", index); 
    srcpad = gst_element_get_static_pad (campipe.vid_enc_bin, "src");
    if (!srcpad) {
        NVGST_ERROR_MESSAGE_V ("can't get static src pad of camera bin %d", index);
        goto fail;
    }
    gst_element_add_pad (campipe.cam_bin, gst_ghost_pad_new ("src", srcpad));
    gst_object_unref (GST_OBJECT (srcpad));

    return TRUE; 

fail:
    app->return_value = -1;
    return FALSE;
}

// nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,framerate=(fraction)30/1" ! 
// nvvidconv flip-method=0 left=0 top=0 right=1920 bottom=1080 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,pixel-aspect-ratio=1/1" ! 
// queue ! nvv4l2h264enc maxperf-enable=true bitrate=8000000 ! h264parse ! queue ! muxer.video_0 

// nvarguscamerasrc sensor-id=1 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,framerate=(fraction)30/1" ! 
// nvvidconv flip-method=0 left=0 top=0 right=1920 bottom=1080 ! "video/x-raw(memory:NVMM),format=(string)NV12,width=(int)1920,height=(int)1080,pixel-aspect-ratio=1/1" ! 
// queue ! nvv4l2h264enc maxperf-enable=true bitrate=8000000 ! h264parse !  queue ! muxer.video_1 

// qtmux name=muxer ! filesink location="test_pair.mp4image" sync=false async=false


static gboolean create_dual_capture_pipeline()
{
    GstBus *bus = NULL;
    GstPad *srcpad = NULL;
    GstPad *sinkpad = NULL;
    gchar sink_x[64];

    // Create the pipeline 
    NVGST_INFO_MESSAGE("create gstreamer pipeline");
    app->gst.pipeline = gst_pipeline_new ("dual_capture_pipeline");;
    if (!app->gst.pipeline) {
        NVGST_ERROR_MESSAGE ("dual capture pipeline creation failed");
        goto fail;
    }

    NVGST_INFO_MESSAGE("watch bus of pipeline");
    bus = gst_pipeline_get_bus (GST_PIPELINE (app->gst.pipeline));
    gst_bus_set_sync_handler (bus, bus_sync_handler, app->gst.pipeline, NULL);
    gst_bus_add_watch (bus, bus_callback, NULL);
    gst_object_unref (bus);

    NVGST_INFO_MESSAGE("set message forward for pipeline");
    g_object_set (app->gst.pipeline, "message-forward", TRUE, NULL);

    // Create the video file sink bin 
    NVGST_INFO_MESSAGE("create video sink bin");
    if (!create_vid_sink_bin()) {
        NVGST_ERROR_MESSAGE ("video sink bin creation failed");
        goto fail;
    }
    gst_bin_add(GST_BIN (app->gst.pipeline), app->gst.vid_sink_bin);

    // Create dual cameras bins 
    for (int index = 0; index < app->num_cams; index++) { 
        NVGST_INFO_MESSAGE_V ("create camera bin %d", index);
        if (!create_csi_cam_bin(index)) {
            NVGST_ERROR_MESSAGE ("camera bin creation failed");
            goto fail;
        }
        gst_bin_add(GST_BIN (app->gst.pipeline), app->gst.cams[index].cam_bin);

        // Manually link camera bin to video sink
        NVGST_INFO_MESSAGE_V("link camera bin & video sink bin %d", index);
        srcpad = gst_element_get_static_pad (app->gst.cams[index].cam_bin, "src");
        if (!srcpad) {
            NVGST_ERROR_MESSAGE_V ("fail to get src pad from camera bin %d", index);
            goto fail;
        }
        g_snprintf (sink_x, sizeof (sink_x), "video_%d", index);
        sinkpad = gst_element_get_static_pad (app->gst.vid_sink_bin, sink_x);
        if (!sinkpad) {
            NVGST_ERROR_MESSAGE_V ("fail to get %s pad from video sink bin %d", sink_x, index);
            goto fail;
        }
        if (GST_PAD_LINK_OK != gst_pad_link (srcpad, sinkpad)) {
            NVGST_ERROR_MESSAGE_V ("fail to link camera bin & video sink bin %d", index);
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
static gboolean create_capture_pipeline()
{
    // Check for capture parameters 
    NVGST_INFO_MESSAGE("check params"); 
    if (!check_capture_params ()) {
        NVGST_ERROR_MESSAGE ("invalid params");
        goto fail;
    }

    // Create pipeline for csi cameras 
    NVGST_INFO_MESSAGE("create pipeline for dual cameras"); 
    if (!create_dual_capture_pipeline ()) {
        NVGST_ERROR_MESSAGE ("can't create pipeline");
        goto fail;
    }

    // Capture pipeline created, now start capture 
    NVGST_INFO_MESSAGE("start pipeline"); 
    GST_INFO_OBJECT (app->gst.pipeline, "camera ready");
    if (GST_STATE_CHANGE_FAILURE == gst_element_set_state (app->gst.pipeline, GST_STATE_PLAYING)) {
        NVGST_CRITICAL_MESSAGE ("can't set pipeline to playing");
        goto fail;
    }

    /* Dump Capture - Playing Pipeline into the dot file
    * Set environment variable "export GST_DEBUG_DUMP_DOT_DIR=/tmp"
    * Run nvgstcapture and 0.00.00.*-nvgstcapture-playing.dot file will be generated.
    * Run "dot -Tpng 0.00.00.*-nvgstcapture-playing.dot > image.png"
    * image.png will display the running capture pipeline.
    */
    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS (GST_BIN(app->gst.pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "nvgstcapture-playing");

    return TRUE;

fail:
    app->return_value = -1;
    return FALSE;
}

// Destroy pipeline.
void destroy_capture_pipeline()
{
    GstPad *sinkpad = NULL;

    // stop the pipeline
    if (!app->gst.pipeline) return;
    if (GST_STATE_CHANGE_FAILURE == gst_element_set_state (app->gst.pipeline, GST_STATE_NULL)) {
        g_warning ("can't set camera pipeline to null\n");
    }

    if (app->reset_thread) {
        g_thread_unref (app->reset_thread);
        app->reset_thread = NULL;
    }

    // remove enc probe 
    for (int i = 0; i < app->num_cams; i++) {
        CamPipe campipe = app->gst.cams[i]; 
        sinkpad = gst_element_get_static_pad (campipe.venc_q, "sink");
        gst_pad_remove_probe (sinkpad, app->venc_probe_id);
        gst_object_unref (sinkpad);

        sinkpad = gst_element_get_static_pad (campipe.ienc_q, "sink");
        gst_pad_remove_probe (sinkpad, app->ienc_probe_id);
        gst_object_unref (sinkpad);
    }
    gst_object_unref (GST_OBJECT (app->gst.pipeline));
}

bool set_capture_params()
{

}

bool start_recording()
{
    set_new_file_name();
    for (int i = 0; i < app->num_cams; i++) {
        CamPipe& campipe = app->gst.cams[i]; 
        g_signal_emit_by_name (G_OBJECT (campipe.cap_tee), "start-capture");
    }
    return true; 
}

bool stop_recording()
{
    for (int i = 0; i < app->num_cams; i++) {
        CamPipe& campipe = app->gst.cams[i]; 
        g_signal_emit_by_name(G_OBJECT(campipe.cap_tee), "stop-capture");
        gst_pad_send_event(gst_element_get_static_pad(campipe.venc_q, "sink"), gst_event_new_eos());
    }
    return true; 
}

int main (int argc, char *argv[])
{
    app = &appctx;
    memset (app, 0, sizeof(AppCtx));
    NVGST_INFO_MESSAGE("init params"); 
    capture_init_params(); 

    NVGST_INFO_MESSAGE("setup interruption"); 
    _intr_setup ();

    NVGST_INFO_MESSAGE("set timer for interruption"); 
    g_timeout_add (400, check_for_interrupt, NULL);

    NVGST_INFO_MESSAGE("init gstreamer"); 
    gst_init (&argc, &argv);
    loop = g_main_loop_new (NULL, FALSE);

    // create the mjpeg server 
    app->stream_server.reset(new mjpeg_server()); 
    if (!app->stream_server) {
        NVGST_WARNING_MESSAGE("failed to create stream server"); 
    }

    // create and run the pipeline 
    NVGST_INFO_MESSAGE("create pipeline"); 
    if (create_capture_pipeline ()) {
        NVGST_INFO_MESSAGE("start gstreamer loop"); 
        g_main_loop_run (loop);
    } 

    // destroy the pipeline 
    NVGST_INFO_MESSAGE("destroy pipeline"); 
    destroy_capture_pipeline ();

    NVGST_INFO_MESSAGE("clean"); 
    g_main_loop_unref (loop);

    // clean 
    // g_mutex_clear (app->lock);
    // g_cond_clear (app->cond);
    // g_free (app->lock);
    // g_free (app->cond);
    // g_free (app->file_name);

    NVGST_INFO_MESSAGE("exit"); 
    return app->return_value; 
}
