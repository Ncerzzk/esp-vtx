#pragma once

#include <cassert>
#include <cstring>

/////////////////////////////////////////////////////////////////////////

constexpr size_t WLAN_INCOMING_BUFFER_SIZE = 1024;
constexpr size_t WLAN_OUTGOING_BUFFER_SIZE = 70000;

////////////////////////////////////////////////////////////////////////////////////

struct Stats
{
    uint32_t wlan_data_sent = 0;
    uint32_t wlan_data_received = 0;
    uint16_t wlan_error_count = 0;
    uint16_t wlan_received_packets_dropped = 0;
    uint32_t video_data = 0;
    uint16_t video_frames = 0;
    uint32_t sd_data = 0;
    uint32_t sd_drops = 0;
};

extern Stats s_stats;


//#define TEST_TIME

#ifdef TEST_TIME
    #define TEST_TIME_FUNC(block,str) do { uint64_t str##_t=esp_timer_get_time(); block; LOG(#str ":%lld us\n",esp_timer_get_time()-str##_t);} while (false) 
#else
    #define  TEST_TIME_FUNC(block,str) block
#endif


