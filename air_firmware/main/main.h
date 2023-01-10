#pragma once

#include <cassert>
#include <cstring>

////////////////////////////////////////////////////////////////////////////////////

//#define TEST_TIME

#ifdef TEST_TIME
    #define TEST_TIME_FUNC(block,str) do { uint64_t str##_t=esp_timer_get_time(); block; LOG(#str ":%lld us\n",esp_timer_get_time()-str##_t);} while (false) 
#else
    #define  TEST_TIME_FUNC(block,str) block
#endif


