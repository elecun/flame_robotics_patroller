#include "serial.hpp"
#include <flame/log.hpp>

namespace flame::device::bus{

    serial::serial(){
        _fd = -1;
    }

    serial::~serial(){
        close();
    }

    bool serial::open(const char* device, const unsigned int baudrate, DataBits databits, ParityBits paritybits, StopBits stopbits) {

        _fd = ::open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if(_fd==-1){ // if the device is not open, return -1
            return false;
        }

        fcntl(_fd, F_SETFL, FNDELAY); // device open in nonblocking mode

        struct termios options;
        tcgetattr(_fd, &options); //get current options
        bzero(&options, sizeof(options)); //clear options

        // baudrate
        speed_t br;
        switch(baudrate){
            case 110 : { br=B110; } break;
            case 300 : { br=B300; } break;
            case 600 : { br=B600; } break;
            case 1200 : { br=B1200; } break;
            case 2400 : { br=B2400; } break;
            case 4800 : { br=B4800; } break;
            case 9600 : { br=B9600; } break;
            case 19200 : { br=B19200; } break;
            case 38400 : { br=B38400; } break;
            case 57600 : { br=B57600; } break;
            case 115200 : { br=B115200; } break;
            default:
                return false;
        }

        // databits
        int databits_flag = 0;
        switch(databits) {
            case DataBits::DATABITS_5: databits_flag = CS5; break;
            case DataBits::DATABITS_6: databits_flag = CS6; break;
            case DataBits::DATABITS_7: databits_flag = CS7; break;
            case DataBits::DATABITS_8: databits_flag = CS8; break;
            default:
                return false;
        }

        // stopbits
        int stopbits_flag = 0;
        switch(stopbits) {
            case StopBits::STOPBITS_1: stopbits_flag = 0; break;
            case StopBits::STOPBITS_2: stopbits_flag = CSTOPB; break;
            default:
                return false;
        }

        // paritybits
        int parity_flag = 0;
        switch(paritybits) {
            case ParityBits::NONE: parity_flag = 0; break;
            case ParityBits::EVEN: parity_flag = PARENB; break;
            case ParityBits::ODD: parity_flag = (PARENB | PARODD); break;
            default:
                return false;
        }

        // set baudreate
        cfsetispeed(&options, br);
        cfsetospeed(&options, br);

        // set options
        options.c_cflag |= ( CLOCAL | CREAD | databits_flag | parity_flag | stopbits_flag);
        options.c_iflag |= ( IGNPAR | IGNBRK );
        options.c_cc[VTIME]=0; //timer unused
        options.c_cc[VMIN]=0; //at least on character before satisfy reading
        tcsetattr(_fd, TCSANOW, &options); //activate settings
        
        return true;
    }

    bool serial::is_opened()
    {
        return _fd>=0;
    }

    void serial::close(){
        ::close(_fd);
        _fd = -1;
    }

    int serial::read(char* buffer, unsigned int max_size, const unsigned int timeout_ms=0, unsigned int duration_us=100){

        timer _time;
        _time.init();

        unsigned int n_read = 0;
        
        while(_time.elapsed()<timeout_ms || timeout_ms==0){
            unsigned char* ptr = (unsigned char*)buffer+n_read;
            int 
        }

    // Timer used for timeout
    timeOut          timer;
    // Initialise the timer
    timer.initTimer();
    unsigned int     NbByteRead=0;
    // While Timeout is not reached
    while (timer.elapsedTime_ms()<timeOut_ms || timeOut_ms==0)
    {
        // Compute the position of the current byte
        unsigned char* Ptr=(unsigned char*)buffer+NbByteRead;
        // Try to read a byte on the device
        int Ret=read(fd,(void*)Ptr,maxNbBytes-NbByteRead);
        // Error while reading
        if (Ret==-1) return -2;

        // One or several byte(s) has been read on the device
        if (Ret>0)
        {
            // Increase the number of read bytes
            NbByteRead+=Ret;
            // Success : bytes has been read
            if (NbByteRead>=maxNbBytes)
                return NbByteRead;
        }
        // Suspend the loop to avoid charging the CPU
        usleep (sleepDuration_us);
    }
    // Timeout reached, return the number of bytes read
    return NbByteRead;
    }

    int serial::write(const char* data, const unsigned int len){

    }

    void serial::flush(){

    }

    int serial::available(){

    }

} // namespace 
