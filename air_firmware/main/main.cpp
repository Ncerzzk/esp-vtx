//#include <Arduino.h>
#include <algorithm>

#include "esp_camera.h"
//#include "EEPROM.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_wifi_types.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
//#include "esp_wifi_internal.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_private/wifi.h"
#include "esp_task_wdt.h"
//#include "bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sodium.h"
#include "esp_wb.hpp"
#include "fec.h"

#include "driver/gpio.h"
#include "main.h"
#include "packets.h"
#include "safe_printf.h"
#include "structures.h"
#include "crc.h"
#include "circular_buffer.h"
#include "pin.h"
#include "wifi.h"


typedef enum{
    IDLE,
    RUN,
    TEST_IMAGE_SIZE,
    TEST_DATA_CORRECT
}camera_state_t;
//#define WIFI_AP

TaskHandle_t transmit_task_handler;
TaskHandle_t framerate_control_task_handler;
TaskHandle_t test_data_task_handler;
QueueHandle_t transmit_data_ready_queue;
Ground2Air_Data_Packet s_ground2air_data_packet;
Ground2Air_Config_Packet s_ground2air_config_packet;
int16_t FRAME_RATE_SET=6;
int16_t LAST_FRAME_RATE_SET=FRAME_RATE_SET;

static uint32_t frame_data_size = 0;
static bool s_video_frame_started = false;
static bool buffer_ready=true;

static uint8_t image_size_test_cnt=0;
camera_state_t camera_state;


/////////////////////////////////////////////////////////////////////////

static int s_uart_verbose = 1;

#define LOG(...) do { if (s_uart_verbose > 0) SAFE_PRINTF(__VA_ARGS__); } while (false) 

/////////////////////////////////////////////////////////////////////////

static constexpr gpio_num_t STATUS_LED_PIN = GPIO_NUM_33;
static constexpr uint8_t STATUS_LED_ON = 0;
static constexpr uint8_t STATUS_LED_OFF = 1;

void initialize_status_led()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << STATUS_LED_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
}

IRAM_ATTR uint64_t micros()
{
    return esp_timer_get_time();
}

IRAM_ATTR uint64_t millis()
{
    return esp_timer_get_time() / 1000ULL;
}

IRAM_ATTR void set_status_led_on()
{
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
}

IRAM_ATTR void update_status_led()
{
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
}

/////////////////////////////////////////////////////////////////////

uint32_t test_image_size(){
    uint32_t image_size;
    image_size_test_cnt=5;
    camera_state = TEST_IMAGE_SIZE;

    while(camera_state==TEST_IMAGE_SIZE){
        vTaskDelay(100);
    };
    image_size = frame_data_size/5;
    return image_size;
}

void test_data_generate(uint8_t * data, size_t len){
    static uint8_t cnt=0;
    uint8_t temp;
    temp=cnt;
    static uint8_t frame_cnt=0;
    data[0] =  frame_cnt;
    for(int i=1;i<len;++i){
        data[i]=temp;
        temp+=3;
    }
    cnt++;
    frame_cnt= (frame_cnt+1) % 14;
}

void test_data_generate2(uint8_t * data, size_t len){
    static uint8_t cnt=0;
    uint8_t temp;
    temp=cnt;
    for(int i=0;i<len;++i){
        data[i]=temp;
        temp+=1;
    }
    cnt=(cnt+1)%10;
}

void static_test_data_generate(uint8_t * data,size_t len){
    for(int i=0;i<len;++i){
        data[i]=i;
    }
}

static void IRAM_ATTR test_data_task(void *pvParameters){
    size_t pushd_size;
    bool compeleted=false;

    uint8_t * ptr=nullptr;
    uint32_t len=0;
    uint8_t index=0;


    while(!transmitter){vTaskDelay(100);}

    printf("test data task start!\n");
    while(true){
        while(!compeleted){
            if(index!=13){
                len=480;
            }else{
                len=480;//esp_random()>>(32-9);
            }
            //ptr=transmitter->push(len,&pushd_size,&compeleted);
            ptr=transmitter->start_push();
            if(ptr){
                test_data_generate(ptr,len);
                transmitter->end_push(len,&compeleted);
            }
            else{
                printf("full\n");
            }
            index=(index+1)%14;
        }
        compeleted=false;
        xQueueSend(transmit_data_ready_queue,&pushd_size,0);
        vTaskDelay(10);
   }
}

IRAM_ATTR static void camera_data_available(const void* data, size_t stride, size_t count, bool last)
{
    static bool image_size_test_start=false;
    static uint8_t jump_frame_cnt=0;
    bool completed=false;
    size_t packet_size=0;
    uint8_t* ptr=nullptr;
    if (data == nullptr ) //start frame
    {   
        if(transmitter &&  camera_state==RUN ){
            s_video_frame_started = true;
        }
        if(camera_state==TEST_IMAGE_SIZE){
            image_size_test_start = true;
        }    
    }
    else 
    {
        const uint8_t* src = (const uint8_t*)data;

        // if (last) //find the end marker for JPEG. Data after that can be discarded
        // {
        //     const uint8_t* dptr = src + (count - 2) * stride;
        //     while (dptr > src)
        //     {
        //         if (dptr[0] == 0xFF && dptr[stride] == 0xD9)
        //         {
        //             count = (dptr - src) / stride + 2; //to include the 0xFFD9
        //             break;
        //         }
        //         dptr -= stride;
        //     }
        // }

        
        if(!jump_frame_cnt  && s_video_frame_started){
            //ptr=transmitter->push(count,&packet_size,&completed);
            ptr=transmitter->start_push();
            if(!ptr){
                printf("full!\n");
            }else{
                for(int i=0; i<count; ++i){
                    *ptr++ = *src; src += stride;
                }
                if(last){
                    ptr-=2;
                    *ptr++=0xff;
                    *ptr++=0xd9;
                }
                transmitter->end_push(count,&completed);
            }
            
            if(completed){
                if(xQueueSend(transmit_data_ready_queue,&packet_size,0)!=pdPASS){
                    printf("failpush!\n");
                }
            }
        }

        if(image_size_test_start && camera_state == TEST_IMAGE_SIZE){
            //memcpy(Frame_buffer+frame_bytes_cnt,CAMERA_BUFFER,count);
            frame_data_size += count;
            if(last){
                image_size_test_cnt--;
                if(image_size_test_cnt==0){
                    camera_state=IDLE;
                }
                image_size_test_start=false;
            }
        }

        if(last && s_video_frame_started ){
            if(ptr && !completed && !jump_frame_cnt){
                while(!completed){ 
                    // we have achieved the end of the frame, while the fragment is not closed yet
                    // push some 0 size packet to make it close
                    //transmitter->push(0,&packet_size,&completed); 
                    ptr=transmitter->start_push();
                    ptr[count-2]=0xff;
                    ptr[count-1]=0xd9;
                    transmitter->end_push(count,&completed);
                }
                xQueueSend(transmit_data_ready_queue,&packet_size,0);
                
            }
            s_video_frame_started=false;

            if(LAST_FRAME_RATE_SET != FRAME_RATE_SET){
                jump_frame_cnt=0;
                LAST_FRAME_RATE_SET = FRAME_RATE_SET;
            }
            jump_frame_cnt=(jump_frame_cnt+1)%FRAME_RATE_SET;
        }
    }

}

static void init_camera()
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_HQVGA;
    config.jpeg_quality = 4;
    config.fb_count = 3;

    // camera init
    esp_err_t err = esp_camera_init(&config, camera_data_available);
    if (err != ESP_OK)
    {
        LOG("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    //s->set_framesize(s, FRAMESIZE_HQVGA);
    s->set_saturation(s, 0);
}

//#define SHOW_CPU_USAGE
static void IRAM_ATTR transmit_task(void *pvParameters){
    size_t packet_size;
    static uint8_t cnt=0;
    while(true){
        if(xQueueReceive(transmit_data_ready_queue, &packet_size, portMAX_DELAY) == pdTRUE) {
            transmitter->do_send_packet(packet_size);
            cnt++;
            if(cnt==30){
                cnt=0;
                transmitter->send_session_key();
            }
        }
    }
}

static void IRAM_ATTR framerate_control_task(void *pvParameters){
    uint32_t distance_set=15;
    static float last_err;
    static float err_i;
    float err,err_d;
    float  kp,kd;

    kp = 0.1f;
    kd = 0.1f;
    
    while(!transmitter){
        vTaskDelay(100);
    }
    while(true){
        float distance=transmitter->get_buffer_available_size();
        err=distance_set-distance;
        err_d=err-last_err;
        err_i += err;
        float pid_out= err*kp + err_d * kd;

        FRAME_RATE_SET +=(int)pid_out;

        if(FRAME_RATE_SET <= 1 ){
            FRAME_RATE_SET = 1;
        }

        FRAME_RATE_SET=1;

        printf("dis:%f\n",distance);
        vTaskDelay(10);
    }
}


extern "C" void app_main()
{
    srand(esp_timer_get_time());
    //configure_uarts();

    transmit_data_ready_queue=xQueueCreate(32, sizeof(size_t));

    printf("Initializing...\n");

    printf("MEMORY at start: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    raw_wifi_init();
    init_camera();
    
    xTaskCreatePinnedToCore(&transmit_task, "transmit_task", 4096, NULL, 9, &transmit_task_handler,0);


    esp_camera_fb_get(); //this will start the camera capture

    //float buffer_size=(float)test_image_size() * 1.5f;
    //printf("buffer size:%f\n",buffer_size);
    printf("MEMORY Before Transmitter: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
    transmitter = new Transmitter(5,8,0,(size_t) 9600,480);

    //xTaskCreate(&framerate_control_task, "FR_C_task", 4096, NULL, 5, &framerate_control_task_handler);
    for(int i=0;i<10;++i){
        transmitter->send_session_key();
    }
    
    esp_log_level_set("esp_wb", ESP_LOG_ERROR);
    camera_state=RUN;

    if(camera_state==TEST_DATA_CORRECT){
        xTaskCreatePinnedToCore(&test_data_task, "TEST_DATA_TASK", 4096, NULL, 4, &test_data_task_handler,0);
    }

    printf("MEMORY Before Loop: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);
    while (true)
    {
        esp_task_wdt_reset();
        vTaskDelay(100);
    }
}
