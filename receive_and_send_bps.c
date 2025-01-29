#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>

#define SERIAL_PORT "/dev/ttyS0"       // Change this to the correct serial port
#define BAUD_RATE B2400                   // Set the baud rate
#define OUTPUT_FILE "received_data.txt"   // File to store received data

// Function to configure the serial port
int configureSerialPort(int fd) {
    struct termios options;

    // Get current configuration
    if (tcgetattr(fd, &options) < 0) {
        perror("tcgetattr failed");
        return -1;
    }

    // Set baud rate
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);

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

    // Apply the configuration
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}

// Function to calculate bytes per second
double calculateBps(long bytes_received, struct timeval start_time) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    // Calculate elapsed time in seconds
    long seconds = current_time.tv_sec - start_time.tv_sec;
    long microseconds = current_time.tv_usec - start_time.tv_usec;

    // If time is in the future (microseconds), adjust the calculation
    if (microseconds < 0) {
        seconds--;
        microseconds += 1000000;
    }

    double elapsed_time = seconds + microseconds / 1000000.0;

    if (elapsed_time == 0) {
        return 0.0; // Avoid division by zero if no time has passed
    }

    // Return bytes per second
    return (double)bytes_received / elapsed_time;
}

int main() {
    // Open the serial port for reading (and writing)
    int fd_serial = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_serial == -1) {
        perror("Failed to open serial port");
        return -1;
    }

    // Configure the serial port
    if (configureSerialPort(fd_serial) != 0) {
        close(fd_serial);
        return -1;
    }

    // Open the file to store received data
    FILE *file = fopen(OUTPUT_FILE, "w");  // Open file in write mode
    if (file == NULL) {
        perror("Failed to open file");
        close(fd_serial);
        return -1;
    }

    unsigned char byte;
    int total_bytes_received = 0;

    // Get the start time for calculating Bps
    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    // Read byte-by-byte from the serial port and save to file
    printf("Receiving data...\n");
    while (1) {
        int bytes_read = read(fd_serial, &byte, 1);
        if (bytes_read > 0) {
            // Write the received byte to the file
            fwrite(&byte, 1, 1, file);
            total_bytes_received++;

            // Calculate bytes per second (Bps)
            double bps = calculateBps(total_bytes_received, start_time);

            // Print bytes per second for each byte received
            printf("Received: %c | Bps: %.2f\n", byte, bps);
        }
        
        // Add a condition to break the loop (e.g., after receiving 200 bytes)
        if (total_bytes_received >= 200) {
            break;
        }
    }

    // Close the file after saving the received data
    fclose(file);

    // Now send the saved data back to the PC
    printf("Sending received data back to the PC...\n");

    // Open the file again for reading to send data back
    file = fopen(OUTPUT_FILE, "r");
    if (file == NULL) {
        perror("Failed to open file for reading");
        close(fd_serial);
        return -1;
    }

    // Send the contents of the file back byte-by-byte
    while (fread(&byte, 1, 1, file) == 1) {
        if (write(fd_serial, &byte, 1) == -1) {
            perror("Failed to write to serial port");
            fclose(file);
            close(fd_serial);
            return -1;
        }
        // Optionally, print each byte being sent back
        printf("Sent: %c\n", byte);
    }

    // Close the file and serial port after the transmission
    fclose(file);
    close(fd_serial);

    printf("Data transmission complete.\n");
    return 0;
}

