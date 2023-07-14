#include <modbus/modbus.h>
#include <thread>
#include <iostream>


#define cmd_speed_reg 0x2199
#define eta_speed_reg 0x219B
#define lfrd_reg 0x219A
#define lfr_reg 0x2136

#define roller_len 0.052*3.14159265359

class atv12
{

public:
atv12(std::string serial_filepath,int device_id = 111,int baud_rate = 19200, char parity = 'E' ,int bits=8,int stop_bits=1);

int connect();

int init();

int exit();

void updateDrive();

void setRef(float freq);

void setLinear(float speed);

int close();

private:
std::string serial_filepath_;
const char* serial_chr;
modbus_t *serial;
std::thread drive_thread;


int freq_ref,prev_ref,rc;
bool running;



};

