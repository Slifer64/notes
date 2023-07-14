#include <unistd.h>
#include <modbus/modbus.h>
#include <atv12.h>
#include <string.h>

atv12::atv12(std::string serial_filepath,int device_id,int baud_rate,char parity,int bits,int stop_bits){

    serial_filepath_ = serial_filepath;
    // serial_chr = new char[serial_filepath.length() + 1];
    // std::copy(serial_filepath.begin(), serial_filepath.end(), serial_chr);
    serial_chr = serial_filepath_.data();

    serial  = modbus_new_rtu(serial_chr, baud_rate, parity, bits, stop_bits);

    if (serial == NULL) {
       fprintf(stderr, "Unable to create the libmodbus context\n");       
    }

    timeval t;
    t.tv_sec = 5;
    t.tv_usec = 0;

    modbus_set_slave(serial, device_id);
    #if __GNUC__ >  7
        modbus_set_response_timeout(serial, t.tv_sec, t.tv_usec ); // timeval struct is now depricated
    #else
        modbus_set_response_timeout(serial, &t); // timeval struct is now depricated
    #endif
    modbus_rtu_set_serial_mode(serial, MODBUS_RTU_RS485);
    modbus_set_debug(serial,FALSE);
    modbus_flush(serial);    

    freq_ref = 0;
    prev_ref = 0;
    running = false;

    }


int atv12::connect(){

    std::cout << serial_filepath_ << std::endl;
    if (modbus_connect(serial) == -1) {
    fprintf(stderr, "Connection atv12::connect() failed: %s\n", modbus_strerror(errno));
    modbus_free(serial);
    return -1;
    }
    return 1;

}
int atv12::init(){

    rc = modbus_write_register(serial, cmd_speed_reg, 0x0080);
    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }
    usleep(100000);
    // CMD 0x0006 NST -> RDY TO SW ON
    rc = modbus_write_register(serial, cmd_speed_reg, 0x0006);
    if (rc == -1) {
      fprintf(stderr, "%s\n", modbus_strerror(errno));
     return -1;
    }   

    usleep(10000);
    // CMD 0x0006 SW ON
    rc = modbus_write_register(serial, cmd_speed_reg, 0x0007);
    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }

    running = true;
    drive_thread = std::thread(&atv12::updateDrive, this);

    return 1;
    }

int atv12::exit(){

    // int rc;
    // uint16_t tab_reg[64];

    running = false;
    drive_thread.join();

    rc = modbus_write_register(serial, lfr_reg, 0);
    if (rc == -1) {
    fprintf(stderr, "%s\n", modbus_strerror(errno));
    return -1;
    }   
    usleep(100000);
    rc = modbus_write_register(serial, cmd_speed_reg, 0x0007);
    if (rc == -1) {
    fprintf(stderr, "%s\n", modbus_strerror(errno));
    return -1;
    }
    usleep(100000);
    rc = modbus_write_register(serial, cmd_speed_reg, 0x0006);
    if (rc == -1) {
    fprintf(stderr, "%s\n", modbus_strerror(errno));
    return -1;
    }
    usleep(100000);

    modbus_flush(serial);
    modbus_close(serial);
    modbus_free(serial);

    return 0;

    }

void atv12::setRef(float freq){

    freq_ref = (int)(freq*10);

}

void atv12::setLinear(float speed){

    //Interpolation of table 
    //Could be be very inaccurate
    float freq;
    if(speed < 0.1136){
        freq = speed * 10 / 0.1136;
    }else if(speed < 0.1756){
        freq = 10 + (speed-0.1136)*(15 - 10)/(0.1756-0.1136);
    }else if(speed < 0.4345){
        freq = 15 + (speed-0.1756)*(20 - 15)/(0.4345-0.1756);
    }else {
        freq = 20 + (speed-0.4345)*(30 - 20)/(0.6450-0.4345);    
    }
    this->setRef(freq);
    

}

void atv12::updateDrive(){

while(running){   
    usleep(100000);
    int rc;
    uint16_t tab_reg[64];
    int freq_tmp = freq_ref;

    rc = modbus_read_registers(serial, eta_speed_reg, 1, tab_reg);
    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));    
    }
    //for (int i=0; i < rc; i++) {
    //    printf("reg[%d]=%d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
    ///}
    usleep(10000);
    if(freq_tmp!=prev_ref){
        std::cout << "Setting speed ref: " << freq_tmp/10 << " hz " << std::endl;        

            rc = modbus_write_register(serial, lfr_reg, freq_tmp);
            if (rc == -1) {
                fprintf(stderr, "%s\n", modbus_strerror(errno));        
            }
            
        if(prev_ref == 0){
            usleep(10000);
            // START DRIVE
            //std::cout << "START DRIVE" << freq_ref << std::endl;
            rc = modbus_write_register(serial, cmd_speed_reg, 0x000F);
            if (rc == -1) {
                fprintf(stderr, "%s\n", modbus_strerror(errno));        
            }

        } 

        if(freq_tmp==0){
            usleep(10000);
            // START DRIVE
            //std::cout << "START DRIVE" << freq_ref << std::endl;
            rc = modbus_write_register(serial, cmd_speed_reg, 0x0007);
            if (rc == -1) {
                fprintf(stderr, "%s\n", modbus_strerror(errno));        
            }

        }     

        
    }

    prev_ref = freq_tmp;
    
}

}