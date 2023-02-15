#include "Comms.h"
#include <iostream>
#include <string>
#include <deque>
#include <mutex>
#include <algorithm>
#include <cstdio>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include "Clock.h"
#include "IHAL.h"
#include "PI_HAL.h"
#include "imgui.h"
#include "HUD.h"
#include "Log.h"
#include "Video_Decoder.h" 
#include "crc.h"
#include "packets.h"
#include <thread>
#include "main.h"
#include <turbojpeg.h>

#include <sys/socket.h>
#include <sys/un.h>
#include "stdint.h"


extern "C"
{
}

/*

Changed on the PI:

- Disable the compositor from raspi-config. This will increase FPS
- Change from fake to real driver: dtoverlay=vc4-fkms-v3d to dtoverlay=vc4-kms-v3d

*/

std::unique_ptr<IHAL> s_hal;
Comms s_comms;

/* This prints an "Assertion failed" message and aborts.  */
void __assert_fail(const char* __assertion, const char* __file, unsigned int __line, const char* __function)
{
    printf("assert: %s:%d: %s: %s", __file, __line, __function, __assertion);
    fflush(stdout);
    //    abort();
}

static std::thread s_comms_thread;

static std::mutex s_ground2air_config_packet_mutex;
static Ground2Air_Config_Packet s_ground2air_config_packet;
using namespace std;
static int sock;
string string_format(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    size_t size = vsnprintf(nullptr, 0, format, args) + 1; // Extra space for '\0'
    va_end(args);
    unique_ptr<char[]> buf(new char[size]);
    va_start(args, format);
    vsnprintf(buf.get(), size, format, args);
    va_end(args);
    return string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

int socket_init(){
        struct sockaddr_un saddr;
        struct sockaddr_un saddr_remote; 
        int fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
        if (fd < 0) throw std::runtime_error(string_format("Error opening socket: %s", strerror(errno)));

        bzero((char *) &saddr, sizeof(saddr));
        saddr.sun_family = AF_UNIX;
        strcpy(saddr.sun_path, "/home/ncer/testsocket");
        unlink("/home/ncer/testsocket");
        int len = strlen(saddr.sun_path) + sizeof(saddr.sun_family);
        if( bind(fd, (struct sockaddr*)&saddr, len)  != 0)
        {
            throw std::runtime_error(string_format("Connect error: %s", strerror(errno)));
        }

        if( listen(fd, 5) != 0 )
        {
            throw std::runtime_error(string_format("Connect error: %s", strerror(errno)));
        }



        uint32_t temp=0;
        int sockfd;
        if( (sockfd = accept(fd, (struct sockaddr*)&saddr_remote, &temp)) != -1 ){
            printf("connected!\n");
        }
        return sockfd;
}

uint8_t raw_data[800*800*3];


inline bool jpeg_decode(uint8_t * data,uint32_t len){
    int width, height;
    int inSubsamp, inColorspace;

    tjhandle tjInstance = tjInitDecompress();
    if (tjDecompressHeader3(tjInstance, data, len, &width, &height, &inSubsamp, &inColorspace) < 0){
        tjDestroy(tjInstance);
        printf("Jpeg header error: %s\n", tjGetErrorStr());
        return false;
    }

    printf("decode jpeg, width:%d height:%d\n",width,height);

    int flags = TJ_FASTUPSAMPLE | TJFLAG_FASTDCT;

    if (tjDecompress2(tjInstance, data, len, raw_data, width, 0/*pitch*/, height, TJPF_BGR, flags) < 0){
        //tjDestroy(m_impl->tjInstance);
        printf("decompressing JPEG image: %s\n", tjGetErrorStr());
        return false;
    }

    return true;
}


static void comms_thread_proc()
{
    Clock::time_point last_stats_tp = Clock::now();
    Clock::time_point last_comms_sent_tp = Clock::now();
    uint8_t last_sent_ping = 0;
    Clock::time_point last_ping_sent_tp = Clock::now();
    Clock::duration ping_min = std::chrono::seconds(999);
    Clock::duration ping_max = std::chrono::seconds(0);
    Clock::duration ping_avg = std::chrono::seconds(0);
    size_t ping_count = 0;

    size_t total_data = 0;
    int16_t min_rssi = 0;

    std::vector<uint8_t> video_frame;
    uint32_t video_frame_index = 0;
    uint8_t video_next_part_index = 0;

    struct RX_Data
    {
        std::array<uint8_t, AIR2GROUND_MTU> data;
        size_t size;
        int16_t rssi = 0;
    };

    RX_Data rx_data;

    while (true)
    {
        //receive new packets
        do
        {
            s_comms.process();
            if (!s_comms.receive(rx_data.data.data(), rx_data.size))
            {
                std::this_thread::yield();
                break;
            }

            rx_data.rssi = (int16_t)s_comms.get_input_dBm();

            //filter bad packets
            Air2Ground_Header& air2ground_header = *(Air2Ground_Header*)rx_data.data.data();
            if (air2ground_header.type != Air2Ground_Header::Type::Video)
            {
                LOGE("Unknown air packet: {}", air2ground_header.type);
                break;
            }

            uint32_t video_packet_size = air2ground_header.size;
            if (video_packet_size > rx_data.size)
            {
                LOGE("Video frame {}: data too big: {} > {}", video_frame_index, video_packet_size, rx_data.size);
                break;
            }

            if (video_packet_size < sizeof(Air2Ground_Video_Packet))
            {
                LOGE("Video frame {}: data too small: {} > {}", video_frame_index, video_packet_size, sizeof(Air2Ground_Video_Packet));
                break;
            }

            size_t payload_size = video_packet_size - sizeof(Air2Ground_Video_Packet);
            Air2Ground_Video_Packet& air2ground_video_packet = *(Air2Ground_Video_Packet*)rx_data.data.data();
            uint8_t crc = air2ground_video_packet.crc;
            air2ground_video_packet.crc = 0;
            uint8_t computed_crc = crc8(0, rx_data.data.data(), sizeof(Air2Ground_Video_Packet));
            if (crc != computed_crc)
            {
                LOGE("Video frame {}, {} {}: crc mismatch: {} != {}", air2ground_video_packet.frame_index, (int)air2ground_video_packet.part_index, payload_size, crc, computed_crc);
                break;
            }

            total_data += rx_data.size;
            min_rssi = std::min(min_rssi, rx_data.rssi);
            //LOGI("OK Video frame {}, {} {} - CRC OK {}. ", air2ground_video_packet.frame_index, (int)air2ground_video_packet.part_index, payload_size, crc);

            if ((air2ground_video_packet.frame_index + 200 < video_frame_index) ||                 //frame from the distant past? TX was restarted
                (air2ground_video_packet.frame_index > video_frame_index)) //frame from the future and we still have other frames enqueued? Stale data
            {
                //if (video_next_part_index > 0) //incomplete frame
                //   s_decoder.decode_data(video_frame.data(), video_frame.size());

                //if (video_next_part_index > 0)
                //LOGE("Aborting video frame {}, {}", video_frame_index, video_next_part_index);

                video_frame.clear();
                video_frame_index = air2ground_video_packet.frame_index;
                video_next_part_index = 0;
                //LOGI("clear");
            }
            if (air2ground_video_packet.frame_index == video_frame_index && air2ground_video_packet.part_index == video_next_part_index)
            {
                video_next_part_index++;
                size_t offset = video_frame.size();
                video_frame.resize(offset + payload_size);
                memcpy(video_frame.data() + offset, rx_data.data.data() + sizeof(Air2Ground_Video_Packet), payload_size);

                if (video_next_part_index > 0 && air2ground_video_packet.last_part != 0)
                {
                    LOGI("Received frame {}, {}, size {}", video_frame_index, video_next_part_index, video_frame.size());
                    //s_decoder.decode_data(video_frame.data(), video_frame.size());

                    //jpeg_decode(video_frame.data(),video_frame.size());
                    send(sock,video_frame.data(),video_frame.size(),MSG_DONTWAIT);
                    video_next_part_index = 0;
                    video_frame.clear();
                }
            }
        } 
        while (false);
    }
}


int run()
{

    s_comms_thread = std::thread(&comms_thread_proc);

    size_t video_frame_count = 0;
    float video_fps = 0;

    Clock::time_point last_stats_tp = Clock::now();
    Clock::time_point last_tp = Clock::now();
    while (true)
    {
        sleep(1);
    }

    return 0;
}



int main(int argc, const char* argv[])
{
    init_crc8_table();

    Comms::RX_Descriptor rx_descriptor;
    rx_descriptor.coding_k = s_ground2air_config_packet.fec_codec_k;
    rx_descriptor.coding_n = s_ground2air_config_packet.fec_codec_n;
    rx_descriptor.mtu = s_ground2air_config_packet.fec_codec_mtu;
    rx_descriptor.interfaces = {"wlx00127b22ac39"};
    Comms::TX_Descriptor tx_descriptor;
    tx_descriptor.coding_k = 2;
    tx_descriptor.coding_n = 6;
    tx_descriptor.mtu = GROUND2AIR_DATA_MAX_SIZE;
    tx_descriptor.interface = "wlx00127b22ac39";
    if (!s_comms.init(rx_descriptor, tx_descriptor))
        return -1;

    for (const auto& itf: rx_descriptor.interfaces)
    {
        system(fmt::format("iwconfig {} channel 11", itf).c_str());
    }
    printf("init socket .. \n");
    sock=socket_init();
    int result = run();
    return result;
}
