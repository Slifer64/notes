#include <epickRTU.h>
#include <unistd.h>
#include <modbus/modbus.h>
#include <string.h>


RoboticEpic40::RoboticEpic40(std::string serial_filepath,int device_id,int baud_rate,char parity,int bits,int stop_bits)
{	
	serial_chr = new char[serial_filepath.length() + 1];
	std::copy(serial_filepath.begin(), serial_filepath.end(), serial_chr);

	serial  = modbus_new_rtu(serial_chr, baud_rate, parity, bits, stop_bits);

	if (serial == NULL) fprintf(stderr, "Unable to create the libmodbus context\n");

	struct timeval t;
	t.tv_sec = 0;
	t.tv_usec  = 5e5;

	modbus_set_slave(serial, device_id);
	#if __GNUC__ >  7
		modbus_set_response_timeout(serial, t.tv_sec, t.tv_usec);
	#else
		modbus_set_response_timeout(serial, &t);
	#endif
	modbus_rtu_set_serial_mode(serial, MODBUS_RTU_RS485);
	modbus_set_debug(serial,FALSE);
	modbus_flush(serial);    

	running = false;

	command.rATR = 0;
	command.rGTO = 0;
	command.rMOD = 0;
	command.rACT = 0;
	command.rPR  = 0;
	command.rSP  = 0;
	command.rFR  = 0;
}

int RoboticEpic40::connect()
{
	if (modbus_connect(serial) == -1) 
	{
		fprintf(stderr, "Connection RoboticEpic40::connect() failed: %s\n", modbus_strerror(errno));
		modbus_free(serial);
		return -1;
	}
	running = true;
	drive_thread = std::thread(&RoboticEpic40::updateDrive, this);

	return 0;
}

void RoboticEpic40::exit()
{
	int rc;
	uint16_t tab_reg[64];

	running = false;
	drive_thread.join();

	modbus_flush(serial);
	modbus_close(serial);
	modbus_free(serial);
}

void RoboticEpic40::configure(int MOD,int PR,int SR,int FR)
{
	command.rMOD = MOD;    
	command.rPR = PR;
	command.rSP = SR;
	command.rFR = FR;
	target_vacuum = PR;   

}

void RoboticEpic40::grip()
{
	command.rACT = 1;
	command.rGTO = 1;
	command.rPR = target_vacuum;
}

void RoboticEpic40::release()
{
	command.rACT = 1;
	command.rGTO = 1;
	command.rPR = 255;
}

void RoboticEpic40::setCommand(int ATR,int MOD,int GTO,int ATC,int PR,int SP,int FR)
{
	command.rATR = ATR;
	command.rGTO = GTO;
	command.rMOD = MOD;
	command.rACT = ATC;
	command.rPR  = PR;
	command.rSP  = SP;
	command.rFR  = FR;
}

void RoboticEpic40::updateDrive()
{
    uint16_t tab_reg[3];
    uint16_t status_reg[3];
    int rc;
    int count=0;
    while(running)
		{  
			modbus_flush(serial);    
			usleep(2e4);
			rc =  modbus_read_input_registers(serial, OUTPUT_REGISTERS, 3, status_reg);
			
			if(rc == -1) fprintf(stderr, "Read registers failed: %s\n", modbus_strerror(errno)); 
			else
			{
				status.gACT = ( ( (status_reg[0] & 0xFF00) >> 8) >> 0) & 0x01;
				status.gMOD = ( ( (status_reg[0] & 0xFF00) >> 8) >> 1) & 0x03;
				status.gGTO = ( ( (status_reg[0] & 0xFF00) >> 8) >> 3) & 0x01;
				status.gSTA = ( ( (status_reg[0] & 0xFF00) >> 8) >> 4) & 0x03;
				status.gOBJ = ( ( (status_reg[0] & 0xFF00) >> 8) >> 6) & 0x03;
				status.gVAS =   ( (status_reg[0] & 0x00FF) >> 0) & 0x03;
				status.gFLT = ( ( (status_reg[1] & 0xFF00) >> 8) >> 0) & 0x0F;
				status.gPR =      (status_reg[1] & 0x00FF);
				status.gPO =    ( (status_reg[2] & 0xFF00) >> 8);
				count++;

				// if(count == 50)
				// {
				// 	std::cout   << "gACT :" << std::to_string(status.gACT) << " gMOD: " 
				// 							<< std::to_string(status.gMOD) << " gGTO: " << std::to_string(status.gGTO) << " gSTA: " 
				// 							<< std::to_string(status.gSTA) << " gOBJ: " << std::to_string(status.gOBJ) << " gVAS: "
				// 							<< std::to_string(status.gVAS) << " gFLT: " << std::to_string(status.gFLT) << " gPR: "
				// 							<< std::to_string(status.gPR)  << " gPO: "  << std::to_string(status.gPO ) << std::endl;
				// 	count = 0;
				// }
			}

			tab_reg[0] = ( ( command.rACT + (command.rMOD << 1) + (command.rGTO << 3) + (command.rATR << 4) ) << 8 ) + 0;        
			tab_reg[1] = ( 0 << 8 ) +  command.rPR;       
			tab_reg[2] = command.rSP << 8 + command.rFR;
			
			rc =  modbus_write_registers(serial,INPUT_REGISTERS,3,tab_reg);
			if(rc == -1)  fprintf(stderr, "Write registers failed: %s\n", modbus_strerror(errno));
			
			usleep(2e4);          
    }
}





