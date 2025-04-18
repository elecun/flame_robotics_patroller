/*!
\file    serial.h
\brief   Header file of the class serialib. This class is used for communication over a serial device.
\author  Philippe Lucidarme (University of Angers)
\version 2.0
\date    december the 27th of 2019
This Serial library is used to communicate through serial port.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE X CONSORTIUM BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This is a licence-free software, it can be used by anyone who try to build a better world.
*/

/**
 * @brief customized for linux use only and c++ with our code convention
 * 
 */

#ifndef FLAME_DEVICE_COMMON_SERIALIB_HPP_INCLUDED
#define FLAME_DEVICE_COMMON_SERIALIB_HPP_INCLUDED

#if defined (__linux__) || defined(__APPLE__)
    #include <stdlib.h>
    #include <sys/types.h>
    #include <sys/shm.h>
    #include <termios.h>
    #include <string.h>
    #include <iostream>
    #include <sys/time.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <sys/ioctl.h>
#endif

namespace flame::device::bus {

    /**
     * number of serial data bits
     */
    enum class DataBits : int {
        DATABITS_5, /**< 5 databits */
        DATABITS_6, /**< 6 databits */
        DATABITS_7, /**< 7 databits */
        DATABITS_8,  /**< 8 databits */
        DATABITS_16,  /**< 16 databits */
    };

    /**
     * number of serial stop bits
     */
    enum class StopBits : int {
        STOPBITS_1, /**< 1 stop bit */
        STOPBITS_1_5, /**< 1.5 stop bits */
        STOPBITS_2, /**< 2 stop bits */
    };

    /**
     * type of serial parity bits
     */
    enum class ParityBits : int {
        NONE, /**< no parity bit */
        EVEN, /**< even parity bit */
        ODD, /**< odd parity bit */
        MARK, /**< mark parity */
        SPACE /**< space bit */
    };

    // use only POSIX Time
    class timer {
        public:
            timer();
            void init();
            unsigned long elapsed(); //unit : msec
        private:
            struct timeval _prev_time;
    };

    class serial
    {
    public:
        serial();
        virtual ~serial();

        bool open(const char* device, const unsigned int baudrate, DataBits databits = DataBits::DATABITS_8, 
                                                                    ParityBits paritybits = ParityBits::NONE, 
                                                                    StopBits stopbits = StopBits::STOPBITS_1);

        bool is_opened();
        void close();
        
        int write(const char* data, const unsigned int len);
        int read(char* buffer, unsigned int max_size, const unsigned int timeout_ms=0, unsigned int duration_us=100);
        void flush();
        int available();// Return the number of bytes in the received buffer

    protected:
        int _fd { -1 };
    }; //class

} // namespace 

#endif // serial
