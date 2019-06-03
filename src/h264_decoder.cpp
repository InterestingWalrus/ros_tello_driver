#ifndef PIX_FMT_RGB24
#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24
#endif

#ifndef CODEC_CAP_TRUNCATED
#define CODEC_CAP_TRUNCATED AV_CODEC_CAP_TRUNCATED
#endif

#ifndef CODEC_FLAG_TRUNCATED
#define CODEC_FLAG_TRUNCATED AV_CODEC_FLAG_TRUNCATED
#endif

#include "ros_tello_driver/h264_decoder.h"
#include <utility>

typedef unsigned char ubyte;


H264Decoder::H264Decoder()
{
    avcodec_register_all();

    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if(!codec)
    {
       ROS_ERROR("Cannot find H264 Decoder");
       throw ros::Exception("H264 Codec not found, check ffmpeg has G264 support");
    }

    context = avcodec_alloc_context3(codec);
    if(!context)
    {
        ROS_ERROR("Cannot allocate context");
        throw ros::Exception("Context Allocation error");
    }

    int err = avcodec_open2(context, codec, 0);
    if(err < 0)
    {
        ROS_ERROR("Cannot open Coded");
        throw ros::Exception("Cannot open Codec");
    }

    parser = av_parser_init(AV_CODEC_ID_H264);
    if(!parser)
    {
        ROS_ERROR("Parser Error");
        throw ros::Exception("Error initializing parser");
    }

    frame = av_frame_alloc();
    if(!frame)
    {
        ROS_ERROR("Frame Allocation Error");
        throw ros::Exception("Cannot allocate frame");
    }

    #if 1
      packet = new AVPacket;
      if(!packet)
      {
          throw ros::Exception("Cannot allocate packet");
      }
      av_init_packet(packet);

    #endif
       
}

H264Decoder::~H264Decoder()
{
    av_parser_close(parser);
    avcodec_close(context);
    av_free(context);
    av_frame_free(&frame);

    #if 1
      delete packet;
    #endif

}

ssize_t H264Decoder::parse(const ubyte* input_data, ssize_t input_size)
{
    auto nread = av_parser_parse2(parser, context, &packet->data, &packet->size, input_data, input_size, 0, 0, AV_NOPTS_VALUE);
    return nread;
}

bool H264Decoder::is_frame_available() const
{
    return packet->size > 0;
}

const AVFrame& H264Decoder::decode_frame()
{
    int got_picture = 0;
    int len = avcodec_decode_video2(context, frame, &got_picture, packet);
    if(len < 0 || got_picture == 0)
    {
        ROS_ERROR("Could not decode frame");
    }

    return *frame;

}

ConvertRGB24::ConvertRGB24()
{
    rgbFrame = av_frame_alloc();
    if(!rgbFrame)
    {
        ROS_ERROR("Cannot allocae frame");
        throw ros::Exception("Cannot allocate frame");
    }

    context = nullptr;
}

ConvertRGB24::~ConvertRGB24()
{
    sws_freeContext(context);
    av_frame_free(&rgbFrame);
}

const AVFrame& ConvertRGB24::convert(const AVFrame &frame, ubyte* output_rgb)
{
    int w = frame.width;
    int h = frame.height;
    int pixel_format = frame.format;

    context = sws_getCachedContext(context, w, h, (AVPixelFormat)pixel_format, w,h, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL,NULL);
    if(!context)
    {
        ROS_ERROR("RGB24 Convert: Cannot allocate Context");
    } 

    // Setup framergb with out_rgb as external buffer. Also say that we want RGB24 output.
     avpicture_fill((AVPicture*)rgbFrame, output_rgb, AV_PIX_FMT_BGR24, w, h);
    // Do the conversion.
    sws_scale(context, frame.data, frame.linesize, 0, h, rgbFrame->data, rgbFrame->linesize);
    rgbFrame->width = w;
    rgbFrame->height = h;
    return *rgbFrame;

}

std::pair<int, int> width_height(const AVFrame& f)
{
  return std::make_pair(f.width, f.height);
}

int row_size(const AVFrame& f)
{
  return f.linesize[0];
}

int ConvertRGB24::predict_size(int w, int h)
{
  // TODO do we need this?
  return avpicture_fill((AVPicture*)rgbFrame, nullptr, AV_PIX_FMT_BGR24, w, h);
}

void disable_logging()
{
  av_log_set_level(AV_LOG_QUIET);
}