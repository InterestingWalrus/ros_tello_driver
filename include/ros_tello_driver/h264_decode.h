// https://github.com/samirchen/TestFFmpeg/blob/master/tutorial01/tutorial01.c
// #include <libavcodec/avcodec.h>
// #include <libavformat/avformat.h>
// #include <libswscale/swscale.h>
// #include <libavutil/imgutils.h>

extern "C"
{
    #include "libswscale/swscale.h"
    #include "libavutil/imgutils.h"
    #include "libavutil/mem.h"
    #include "libavutil/opt.h"
    #include "libavcodec/avcodec.h"
    #include "libavutil/mathematics.h"
    #include "libavutil/samplefmt.h"
    #include "libavformat/avformat.h"
}

#include <stdio.h>

// compatibility with newer API
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc avcodec_alloc_frame
#define av_frame_free avcodec_free_frame
#endif

#include <cstdlib>
#include <stdexcept>
#include <ros/ros.h>




class H264Decoder
{
    private:
    // Initialize all to NULL to prevent SEGFaults
    AVFormatContext *formatContext = NULL;
    AVCodecContext *context = NULL;
    AVCodecContext *contextOrig = NULL;
    AVCodec *codec = NULL;
    AVFrame *frame = NULL;
    AVFrame *rgb_frame = NULL;
    AVCodecParserContext *parser;

    int frame_size;
    int i;
    int numBytes;
    int got_picture;
    uint8_t *buffer = NULL;
    AVPacket* packet;
    struct SwsContext *sws_context = NULL;

    public:

        H264Decoder();
        ~H264Decoder();

        const AVFrame& decode_frame();
        void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame);
        int return_size();

        /* First, parse a continuous data stream, dividing it into
        packets. When there is enough data to form a new frame, decode
        the data and return the frame. parse returns the number
        of consumed bytes of the input stream. It stops consuming
        bytes at frame boundaries.
        */
        ssize_t parse(const unsigned char* input_data, ssize_t input_size );
         bool is_frame_available() const;



};