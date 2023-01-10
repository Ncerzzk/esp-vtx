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

Transmitter *transmitter=0;
QueueHandle_t fec_data_ready_queue;

static void IRAM_ATTR fec_task(void *pvParameters){
    uint8_t block_id;
    // wait for transmitter initialized
    while(!transmitter){
        vTaskDelay(100);
    }
    while(true){
        if(xQueueReceive(fec_data_ready_queue, &block_id, portMAX_DELAY) == pdTRUE) {
            transmitter->do_fec(block_id);
        }
    }
}

TaskHandle_t fec_task_handler;

// uint8_t push_buffer[MAX_FEC_PAYLOAD*2*5];
// uint8_t fec_buffer[MAX_FEC_PAYLOAD*2*3];

Transmitter::Transmitter(int k, int n, uint8_t radio_port,size_t total_buffer_size,size_t line_size):  fec_k(k), fec_n(n), block_idx(0),
                                                                fragment_idx(0),
                                                                radio_port(radio_port)
{
    uint32_t packet_cnt_per_frame=0;

    combined_cnt = 1400/line_size;

    packet_cnt_per_frame = total_buffer_size/line_size/combined_cnt;
    packet_cnt_per_frame ++;

    fec_p = fec_new(fec_k, fec_n);
    uint8_t fec_fragment_num=fec_n-fec_k;

    block_cnt = 3;//packet_cnt_per_frame / fec_k;
    //block_list = new uint8_t**[block_cnt];
    block_list = new Block *[block_cnt];
    //uint8_t *push_buffer=new uint8_t[MAX_FEC_PAYLOAD*block_cnt*fec_k];
    // we apply a big buffer for push and fec here
    // the continuous buffer has benfit when the write size larger then fragment size
    //uint8_t *fec_buffer=new uint8_t[MAX_FEC_PAYLOAD*block_cnt*fec_fragment_num];


    for(int i=0; i < block_cnt;++i){
        //block_list[i] = new uint8_t*[fec_n];
        block_list[i] = new Block(fec_n);

        for(int j=0; j < fec_n; j++)
        {
            block_list[i]->fragments[j] = new uint8_t[MAX_FEC_PAYLOAD];
        }
        // for(int j=0;j<fec_k;j++){
        //    block_list[i]->fragments[j] = push_buffer + i*MAX_FEC_PAYLOAD*fec_k + j*MAX_FEC_PAYLOAD;
        // }
        // for(int j=fec_k;j<fec_n;j++){
        //    block_list[i]->fragments[j] = fec_buffer + i*MAX_FEC_PAYLOAD*fec_fragment_num + (j-fec_k)*MAX_FEC_PAYLOAD; 
        // }
    }

    ESP_LOGI(TAG,"block_cnt:%u  packet_cnt_per_frame:%u combined_cnt:%d \n",block_cnt,packet_cnt_per_frame,combined_cnt);
    // aim to locate 1 frame in the blocks
    make_session_key();

    fec_data_ready_queue=xQueueCreate(32, sizeof(uint8_t));
    //xTaskCreate(&fec_task, "fec_task", 4096, NULL, 9, &fec_task_handler);
    xTaskCreatePinnedToCore(&fec_task, "fec_task", 4096, NULL, 9, &fec_task_handler,0);

    current_block_idx=block_cnt-1;
    fragment_idx=0;

    push_fragment_idx=0;
    push_block_idx=0;

    pushed_fragment_size=0;

    ESP_LOGI(TAG,"Transmitter Inited!\n");
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
    //memcpy(p, radiotap_header, sizeof(radiotap_header));     
    //p += sizeof(radiotap_header);
    // we don'nt need do this, esp32 api will do it for us

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
    Block &block=*block_list[current_block_idx];
    wblock_hdr_t *block_hdr = (wblock_hdr_t*)ciphertext;
    long long unsigned int ciphertext_len;

    block_hdr->packet_type = WFB_PACKET_DATA;
    block_hdr->nonce = htobe64(((block_idx & BLOCK_IDX_MASK) << 8) + fragment_idx);

    ESP_LOGI(TAG,"send fragment. block_idx:%d   fragment_idx:%d size:%d \n",current_block_idx,fragment_idx,packet_size);

    if(packet_size==0){
        return ;
    }

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
    // eg: push_block_idx = 2, curr_block_idx=1, block_cnt=5,fec_k=8
    // push_fragment=3, frag_idx=2 
    // distance = (5 - 2 + 1 -1)* fec_k = 3 * 8 
    // distance += (8 - 3) + 2 + 1
    if(push_block_idx>current_block_idx){
        distance =(block_cnt - push_block_idx + current_block_idx - 1 )*fec_k;
    }else{
        distance = (current_block_idx - push_block_idx -1)*fec_k;
    }
    distance += fec_k - push_fragment_idx + (fragment_idx > fec_k? fec_k:fragment_idx) ;
    
    return distance;
}

uint8_t * Transmitter::start_push(void){
    if(push_block_idx == current_block_idx ){
        return nullptr;// the buffer is full
    }

    Block &block=*block_list[push_block_idx];
    wpacket_hdr_t *packet_hdr = (wpacket_hdr_t *)(block[push_fragment_idx]);
    uint8_t * result_ptr;

    result_ptr = block[push_fragment_idx] + sizeof(wpacket_hdr_t) + pushed_fragment_size;

    return result_ptr;
}

void Transmitter::end_push(size_t size,bool* completed){
    Block &block=*block_list[push_block_idx];
    wpacket_hdr_t *packet_hdr = (wpacket_hdr_t *)(block[push_fragment_idx]);

    pushed_fragment_size += size;
    fragment_packet_cnt++; 

    packet_hdr->packet_size = htobe16(pushed_fragment_size); // edit the packet_size

    if(fragment_packet_cnt != combined_cnt){
        // TODO: this implement should be changed later.
        // we should use the size to determined the fragment is closed or not.
        *completed=false;
    }else{
        *completed=true;
        // the fragment has been full
        block.send_packet_sizes[push_fragment_idx] = pushed_fragment_size + sizeof(wpacket_hdr_t);
        block.max_packet_size = max(block.max_packet_size,sizeof(wpacket_hdr_t) + pushed_fragment_size);   // TODO: atomic race!

        fragment_packet_cnt=0;
        pushed_fragment_size=0;
        push_fragment_idx++; // change to next fragment

        if(push_fragment_idx == fec_k){
            // the block has been full
            // change to next block, and send signal to fec this block
            push_fragment_idx = 0;
            xQueueSend(fec_data_ready_queue,&push_block_idx,0);
            push_block_idx++;

            if(push_block_idx == block_cnt){
                push_block_idx=0;
            }
        }
    }
}


extern QueueHandle_t transmit_data_ready_queue;
void Transmitter::do_fec(uint8_t block_id){
    Block &block=*block_list[block_id];
    size_t temp=0;
    
    TEST_TIME_FUNC(fec_encode(fec_p, (const uint8_t**)block.fragments, block.fragments + fec_k, block.max_packet_size),FEC);

    for(int i=fec_k; i< fec_n; ++i){
        if(xQueueSend(transmit_data_ready_queue,&temp,0)!=pdPASS){
            printf("error in do fec!\n");
        }
        block.send_packet_sizes[i] = block.max_packet_size;
        // the value of sending event is useless, edit this later
    }
    block.max_packet_size=0;
}

void Transmitter::do_send_packet(size_t size){
    // the parameter size is useless, need to be removed
    Block * block=block_list[current_block_idx];
    send_block_fragment(block->send_packet_sizes[fragment_idx]); 
    fragment_idx++;

    if(fragment_idx == fec_n){
        current_block_idx += 1;
        block_idx += 1;
        fragment_idx = 0;
    }

    if(current_block_idx == block_cnt){
        current_block_idx = 0;
    }

        // Generate new session key after MAX_BLOCK_IDX blocks
    if (block_idx > MAX_BLOCK_IDX)
    {
        make_session_key();
        send_session_key();
        block_idx = 0;
    }


}
