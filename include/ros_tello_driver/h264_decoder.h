/*Adapted from https://github.com/clydemcqueen/tello_ros/blob/master/tello_driver/h264decoder/h264decoder.cpp and 
https://github.com/tilk/h264_image_transport**/

#ifndef H264_DECODER_H
#define H264_DECODER_H


// for ssize_t (signed int type as large as pointer type)
#include <cstdlib>
#include <stdexcept>
#include <ros/ros.h>


extern "C"
{
    #include "libswscale/swscale.h"
    #include "libavutil/imgutils.h"
    #include "libavutil/mem.h"
    #include "libavutil/opt.h"
    #include "libavcodec/avcodec.h"
    #include "libavutil/mathematics.h"
    #include "libavutil/samplefmt.h"
}

class H264Exception : public std::runtime_error
{
public:
  H264Exception(const char* s) : std::runtime_error(s) {}
};

class H264InitFailure : public H264Exception
{
public:
  H264InitFailure(const char* s) : H264Exception(s) {}
};

class H264DecodeFailure : public H264Exception
{
public:
  H264DecodeFailure(const char* s) : H264Exception(s) {}
};

class H264Decoder
{
    private:

        AVCodec *codec;
        AVCodecContext *context;
        AVFrame *frame;
        AVPacket *packet;
        AVCodecParserContext *parser;

    public:

        H264Decoder();
        ~H264Decoder();

        /* First, parse a continuous data stream, dividing it into
        packets. When there is enough data to form a new frame, decode
        the data and return the frame. parse returns the number
        of consumed bytes of the input stream. It stops consuming
        bytes at frame boundaries.
        */
        ssize_t parse(const unsigned char* input_data, ssize_t input_size );
        bool is_frame_available() const;
        const AVFrame& decode_frame();





};

class ConvertRGB24
{
   
   private:

    SwsContext *context;
    AVFrame *rgbFrame;
   

   public:
     ConvertRGB24();
     ~ConvertRGB24();

    /*  Returns, given a width and height,
      how many bytes the frame buffer is going to need. */

    int predict_size(int w, int h);

     /*  Given a decoded frame, convert it to RGB format and fill
    out_rgb with the result. Returns a AVFrame structure holding
    additional information about the RGB frame, such as the number of
    bytes in a row and so on. */

     const AVFrame& convert(const AVFrame &frame, unsigned char* out_rgb);
     
};

void disable_logging();

/* Wrappers, so we don't have to include libav headers. */
std::pair<int, int> width_height(const AVFrame&);
int row_size(const AVFrame&);




#endif