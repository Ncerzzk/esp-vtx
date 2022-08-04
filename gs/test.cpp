
#include <thread>
#include "stdio.h"
#include <unistd.h>

static std::thread s_comms_thread;

void test(){
    printf("hello,world");
}
int main(){
    s_comms_thread = std::thread(&test);
    sleep(5);
    return 0;
}