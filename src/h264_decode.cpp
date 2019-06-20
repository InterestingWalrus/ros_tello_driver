#include "ros_tello_driver/h264_decode.h"
#include <utility>

typedef unsigned char ubyte;

H264Decoder::H264Decoder()
{
     avcodec_register_all();
   // av_register_all(); // deprecated in ffmpeg 4

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
        ROS_ERROR("Cannot open Codec");
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

     rgb_frame = av_frame_alloc();
    if(!rgb_frame)
    {
        ROS_ERROR("RGB Frame Allocation Error");
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
    av_free(context);
    // Free RGB frame
    av_frame_free(&rgb_frame);

    // Free YUV Frame
    av_frame_free(&frame);

    // CLose all codecs:
    avcodec_close(context);
    avcodec_close(contextOrig); 

}

const AVFrame& H264Decoder::decode_frame()
{
    // Determine required buffer size and allocate buffer.
	//numBytes=avpicture_get_size(AV_PIX_FMT_RGB24,context->width, context->height); // Deprecated.
	numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, context->width, context->height, 1);
	buffer = (uint8_t *) av_malloc(numBytes * sizeof(uint8_t));

    // Assign appropriate parts of buffer to image planes in rgb_frame Note that rgb_frame is an AVFrame, but AVFrame is a superset of AVPicture
	frame_size = av_image_fill_arrays(rgb_frame->data, rgb_frame->linesize, buffer, AV_PIX_FMT_RGB24, context->width, context->height, 1);

    // Initialize SWS context for software scaling.
	sws_context = sws_getContext(context->width, context->height, context->pix_fmt, context->width, context->height,
                                AV_PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);

    // read frames and save first 5 to disk
    i = 0;

    int ret;
    
    if(&packet)
    {
        ret = avcodec_send_packet(context, packet);

        // In particular, we don't expect AVERROR(EAGAIN), because we read all
        // decoded frames with avcodec_receive_frame() until done.
        if(ret != 0)
        {
            if(ret == AVERROR_EOF)
            {
                ROS_ERROR ("AV_ERROR_EOF ROS");

            } 
        }

    }

    ret = avcodec_receive_frame(context, frame);

    if(ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
    {
        return *frame;
    }

    if(ret >=0)
    {
        got_picture = 1;
    }

    if(ret == AVERROR(EAGAIN))
    {
        ROS_ERROR ("AV_ERROR(EAGAIN) ROS");
    }

    ROS_INFO("Frame Width = %d, Frame height = %d", frame->width, frame->height);
    
    if(got_picture)
    {
        // Convert the image from its native format to RGB.
            sws_scale(sws_context, (uint8_t const * const *) frame->data, frame->linesize, 0, context->height, 
                        rgb_frame->data, rgb_frame->linesize);
    }

    // Save frame to disk
    if(++i <= 5)
    {
        SaveFrame(rgb_frame, context->width, context->height, i);
    }

    

    // Free the packet
    av_packet_unref(packet);

    return *rgb_frame;
    
}

//TODO Change to ifstream
void H264Decoder::SaveFrame(AVFrame *pFrame, int width, int height, int iFrame)
{
	FILE *pFile;
	char szFilename[32];
	int y;
  
	// Open file.
	sprintf(szFilename, "frame%d.ppm", iFrame);
	pFile = fopen(szFilename, "wb");
	if (pFile == NULL) {
		return;
	}
  
	// Write header.
	fprintf(pFile, "P6\n%d %d\n255\n", width, height);
  
	// Write pixel data.
	for (y = 0; y < height; y++) {
		fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);
	}
  
	// Close file.
	fclose(pFile);
}

int H264Decoder::return_size()
{
    return frame_size;
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
