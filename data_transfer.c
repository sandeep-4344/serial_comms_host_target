#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <time.h>


#define BAUD_RATE B2400
#define MAX_BUF_SIZE 1024
#define INPUT_FILE "received_data.txt"
#define BYTE_LIMIT 1000

int main() {

int bytes_read;
int i=0;
int total_bytes_received = 0;
char buffer[MAX_BUF_SIZE];
//double temp = 0, previous = 0,rate = 0;
unsigned char byte;


    // Open the serial port
    const char *serial_port = "/dev/ttyUSB0";
  //sudo  const char *serial_port_host = "/dev/ttyS0";
    int serial_fd = open(serial_port, O_RDWR);

    if (serial_fd == -1) {
        perror("Unable to open serial port");
        return 1;
    }

    // Set up the serial port configuration
    struct termios options;
    tcgetattr(serial_fd, &options);

    // Set baud rate to 2400, 8 data bits, no parity, 1 stop bit
    cfsetispeed(&options, B2400);
    cfsetospeed(&options, B2400);
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Clear size bits
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cflag &= ~CRTSCTS; // No hardware flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver and set local mode

    tcsetattr(serial_fd, TCSANOW, &options);

    // Send data
    const char *data = "Finance Minister Arun Jaitley Tuesday hit out at former RBI governor Raghuram Rajan for predicting that the next banking crisis would be triggered by MSME lending, saying postmortem is easier than taking action when it was required. Rajan, who had as the chief economist at IMF warned of impending financial crisis of 2008, in a note to a parliamentary committee warned against ambitious credit targets and loan waivers, saying that they could be the sources of next banking crisis. Government shfocus on sources of the next crisis, not just the last one.*";
    
    
    
    int bytes_written = write(serial_fd, data, strlen(data));

    if (bytes_written < 0) {
        perror("Error writing to serial port");
        close(serial_fd);
        return 1;
    }

    printf("Data sent: %s\n", data);
close(serial_fd);

//reconfiguring serial port
  serial_fd = open(serial_port, O_RDWR);

    if (serial_fd == -1) {
        perror("Unable to open serial port");
        return 1;
    }

    // Set up the serial port configuration

    tcgetattr(serial_fd, &options);

    // Set baud rate to 2400, 8 data bits, no parity, 1 stop bit
    cfsetispeed(&options, B2400);
    cfsetospeed(&options, B2400);
 
    
    // Set 8 data bits, no parity, 1 stop bit
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;   // Mask the character size bits
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cflag |= CLOCAL;   // Ignore modem control lines
    options.c_cflag |= CREAD;    // Enable receiver

    // Set the serial port to raw mode (disable input processing)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(serial_fd, TCSANOW, &options);
      
    printf("Listening for data on %s...\n", serial_port);
  while (1) {
        int bytes_read = read(serial_fd, &byte, 1);
        if (bytes_read > 0) {
            
            total_bytes_received++;
            
            // Printing each byte received
            printf("Received: %c \n", byte);
        }
        
        // condition to break the loop
        if (total_bytes_received >= 550) {
            break;
        }
    }
    close(serial_fd);
    return 0;
}
