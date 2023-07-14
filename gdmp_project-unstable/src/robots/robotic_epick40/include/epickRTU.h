#include <modbus/modbus.h>
#include <thread>
#include <iostream>

#define INPUT_REGISTERS 0x03E8
#define OUTPUT_REGISTERS 0x07D0

struct RoboticEpic40Input
{
	uint8_t gACT; 
	uint8_t gMOD;
	uint8_t gGTO;
	uint8_t gSTA;
	uint8_t gOBJ;
	uint8_t gVAS;
	uint8_t gFLT;
	uint8_t gPR;
	uint8_t gPO;
};

struct RoboticEpic40Output
{
	uint8_t rATR;
	uint8_t rGTO;
	uint8_t rMOD;
	uint8_t rACT;
	uint8_t rPR;
	uint8_t rSP;
	uint8_t rFR;
};


class RoboticEpic40
{

public:

	RoboticEpic40(std::string serial_filepath,int device_id = 9,int baud_rate = 115200, char parity = 'N' ,int bits=8,int stop_bits=1);

	int connect();

	int init();

	void exit();

	void updateDrive();

	int close();

	void setCommand(int ATR,int MOD,int GTO,int ATC,int PR,int SP,int FR);

	void configure(int MOD,int PR,int SR,int FR);
	void grip();
	void release();

	int target_vacuum;

private:

	char* serial_chr;
	modbus_t *serial;
	std::thread drive_thread;
	bool running;

	RoboticEpic40Output command;  
	RoboticEpic40Input status;
};