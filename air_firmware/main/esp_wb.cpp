#include "esp_wb.hpp"
#include "wifibroadcast.hpp"
#include "esp_private/wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include "esp_wifi_types.h"
#include "esp_heap_caps.h"
#include "endian.h"
#include "fec.h"
#include "ieee80211_radiotap.h"

#include "key.h"

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
#include "esp_log.h"

#define LOG(...) do { SAFE_PRINTF(__VA_ARGS__); } while (false) 
#define max(a,b) (a>b?a:b)

static const char* TAG = "esp_wb";

Transmitter *transmitter=nullptr;
QueueHandle_t fec_data_ready_queue;

static void IRAM_ATTR fec_task(void *pvParameters){
    uint32_t temp;
    // wait for transmitter initialized
    while(!transmitter){
        vTaskDelay(100);
    }
    while(true){
        if(xQueueReceive(fec_data_ready_queue, &temp, portMAX_DELAY) == pdTRUE) {
            transmitter->do_fec();
        }
    }
}

TaskHandle_t fec_task_handler;


Transmitter::Transmitter(int k, int n, uint8_t radio_port,size_t total_buffer_size,size_t line_size):  fec_k(k), fec_n(n), block_idx(0),
                                                                fragment_idx(0),
                                                                max_packet_size(0),
                                                                radio_port(radio_port)
{
    uint32_t packet_cnt_per_frame=0;

    combined_cnt = 1400/line_size;

    packet_cnt_per_frame = total_buffer_size/line_size/combined_cnt;
    packet_cnt_per_frame ++;

    fec_p = fec_new(fec_k, fec_n);

    block_cnt_per_frame = 5;//packet_cnt_per_frame / fec_k;
    block_list = new uint8_t**[block_cnt_per_frame];

    max_packet_sizes = new size_t[block_cnt_per_frame];

    for(int i=0; i < block_cnt_per_frame;++i){
        block_list[i] = new uint8_t*[fec_n];

        for(int j=0; j < fec_n; j++)
        {
            block_list[i][j] = new uint8_t[MAX_FEC_PAYLOAD];
        }
    }

    ESP_LOGI(TAG,"block_cnt_per_frame:%u  packet_cnt_per_frame:%u combined_cnt:%d \n",block_cnt_per_frame,packet_cnt_per_frame,combined_cnt);
    // aim to locate 1 frame in the blocks
    make_session_key();

    fec_data_ready_queue=xQueueCreate(32, sizeof(size_t));
    xTaskCreate(&fec_task, "fec_task", 4096, NULL, 9, &fec_task_handler);

    current_block_idx=block_cnt_per_frame-1;
    fragment_idx=0;
}

Transmitter::~Transmitter()
{
    // this method should be rewrite 
    // while as for mcu, it's not import to do this.


    // for(int i=0; i < fec_n; i++)
    // {
    //     delete block[i];
    // }
    // delete block;

    // fec_free(fec_p);
}


void Transmitter::make_session_key(void)
{
    randombytes_buf(session_key, sizeof(session_key));
    session_key_packet.packet_type = WFB_PACKET_KEY;
    randombytes_buf(session_key_packet.session_key_nonce, sizeof(session_key_packet.session_key_nonce));
    if (crypto_box_easy(session_key_packet.session_key_data, session_key, sizeof(session_key),
                        session_key_packet.session_key_nonce, (uint8_t *)rx_pubkey, (uint8_t *)tx_secretkey) != 0)
    {
        //throw runtime_error("Unable to make session key!");
        ;
    }
}

void  Transmitter::inject_packet(const uint8_t *buf, size_t size)
{

    uint8_t *p = txbuf;

    // radiotap header
    //memcpy(p, radiotap_header, sizeof(radiotap_header));     // may be optimized later
    //p += sizeof(radiotap_header);

    // ieee80211 header
    memcpy(p, ieee80211_header, sizeof(ieee80211_header));
    p[SRC_MAC_LASTBYTE] = radio_port;
    p[DST_MAC_LASTBYTE] = radio_port;
    p[FRAME_SEQ_LB] = ieee80211_seq & 0xff;
    p[FRAME_SEQ_HB] = (ieee80211_seq >> 8) & 0xff;
    ieee80211_seq += 16;
    p += sizeof(ieee80211_header);

    // FEC data
    memcpy(p, buf, size);
    p += size;
    esp_wifi_80211_tx(WIFI_IF_STA, txbuf,p - txbuf, false);

}


void Transmitter::send_block_fragment(size_t packet_size)
{
    uint8_t** block=block_list[current_block_idx];
    wblock_hdr_t *block_hdr = (wblock_hdr_t*)ciphertext;
    long long unsigned int ciphertext_len;

    //assert(packet_size <= MAX_FEC_PAYLOAD);

    block_hdr->packet_type = WFB_PACKET_DATA;
    block_hdr->nonce = htobe64(((block_idx & BLOCK_IDX_MASK) << 8) + fragment_idx);

    ESP_LOGI(TAG,"send fragment. block_idx:%lld   fragment_idx:%d\n",block_idx,fragment_idx);

    // encrypted payload
    // encrypt 1000 bytes ~= 400 us
    TEST_TIME_FUNC(crypto_aead_chacha20poly1305_encrypt(ciphertext + sizeof(wblock_hdr_t), &ciphertext_len,
                                         block[fragment_idx], packet_size,
                                         (uint8_t*)block_hdr, sizeof(wblock_hdr_t),
                                         NULL, (uint8_t*)(&(block_hdr->nonce)), session_key), ENCRYPT);


    //inject_packet 1000 bytes  ~= 180 us
    TEST_TIME_FUNC(inject_packet(ciphertext, sizeof(wblock_hdr_t) + ciphertext_len),INJECT);

 
    
}

void Transmitter::send_session_key(void)
{
    //fprintf(stderr, "Announce session key\n");
    inject_packet((uint8_t*)&session_key_packet, sizeof(session_key_packet));
}

void Transmitter::clean_cnts(void ){
    fragment_packet_cnt=0;
    pushed_fragment_size=0;
    push_fragment_idx=0;
    push_block_idx=0;


    current_block_idx=0; 
    fragment_idx=0;
}

uint32_t Transmitter::get_buffer_available_size(void){
    uint32_t distance=0;
    // eg: push_block_idx = 2, curr_block_idx=1, block_cnt_per_frame=5,fec_k=8
    // push_fragment=3, frag_idx=2 
    // distance = (5 - 2 + 1 -1)* fec_k = 3 * 8 
    // distance += (8 - 3) + 2 + 1
    if(push_block_idx>current_block_idx){
        distance =(block_cnt_per_frame - push_block_idx + current_block_idx - 1 )*fec_k;
    }else{
        distance = (current_block_idx - push_block_idx -1)*fec_k;
    }
    distance += fec_k - push_fragment_idx + (fragment_idx > fec_k? fec_k:fragment_idx) ;
    
    return distance;
}

uint8_t * Transmitter::push(size_t size,size_t * pushd_size,bool* completed){

    if(push_block_idx == current_block_idx ){
        return nullptr;// the buffer is full
    }

    uint8_t** block=block_list[push_block_idx];
    wpacket_hdr_t *packet_hdr = (wpacket_hdr_t *)block[push_fragment_idx];
    uint8_t * result_ptr;

    result_ptr = block[push_fragment_idx] + sizeof(wpacket_hdr_t) + pushed_fragment_size;
    pushed_fragment_size += size;
    fragment_packet_cnt++;

    *pushd_size=pushed_fragment_size;
    packet_hdr->packet_size = htobe16(pushed_fragment_size); // edit the packet_size

    ESP_LOGI(TAG,"pushd.  push_fragment_idx:%u  push_block_idx:%u fragment_packet_cnt:%d \n",push_fragment_idx,push_block_idx,fragment_packet_cnt);
    if(fragment_packet_cnt != combined_cnt){
        // TODO: this implement should be changed later.
        // we should use the size to determined the fragment is closed or not.
        *completed=false;
        return result_ptr;
    }

    // the fragment has been full
    *completed=true;
    fragment_packet_cnt=0;
    pushed_fragment_size=0;
    push_fragment_idx++; // change to next fragment
    

    if(push_fragment_idx == fec_k){
        // the block has been full
        // change to next block
        push_fragment_idx = 0;
        push_block_idx++;

        if(push_block_idx == block_cnt_per_frame){
            push_block_idx=0;
        }
    }
    
    return result_ptr;
}

extern QueueHandle_t transmit_data_ready_queue;
void Transmitter::do_fec(uint8_t block_id){
    uint8_t** block=block_list[block_id];
    TEST_TIME_FUNC(fec_encode(fec_p, (const uint8_t**)block, block + fec_k, max_packet_size),FEC);
    for(int i=fec_k; i< fec_n; ++i){
        xQueueSend(transmit_data_ready_queue,&max_packet_size,0);
    }
}

void Transmitter::do_send_packet(size_t size){
    uint8_t** block=block_list[current_block_idx];
    ESP_LOGI(TAG,"send packet. cur block:%d push block:%d      cur frag:%d   push frag:%u\n",current_block_idx,push_block_idx,fragment_idx,push_fragment_idx);

    max_packet_size = max(max_packet_size, sizeof(wpacket_hdr_t) + size); 
    if(fragment_idx < fec_k){
        send_block_fragment(sizeof(wpacket_hdr_t) + size); 
    }
    else{
        send_block_fragment(max_packet_size);    
    }

    fragment_idx++;

    if(fragment_idx == fec_k){
        xQueueSend(fec_data_ready_queue,&max_packet_size,0);
    }
    
    if(fragment_idx == fec_n){
        current_block_idx += 1;
        block_idx += 1;
        fragment_idx = 0;
        max_packet_size = 0; 
    }

    if(current_block_idx == block_cnt_per_frame){
        current_block_idx = 0;
        //push_fragment_idx=0;
        //push_block_idx=0;
    }

        // Generate new session key after MAX_BLOCK_IDX blocks
    if (block_idx > MAX_BLOCK_IDX)
    {
        make_session_key();
        send_session_key();
        block_idx = 0;
    }


}
