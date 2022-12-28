// UNIX domain socket code based on example in http://man7.org/linux/man-pages/man7/unix.7.html
// UART code based on example in https://github.com/Digilent/linux-userspace-examples.git
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "uart.h"
#include "connection.h"

static void *control_block;

void esp32_protocol(void *control_block)
{
    struct UartDevice dev;
    int rc;
    size_t offset;
    size_t index;
    u16 pos;
    size_t read_data_len;
    size_t const tx_buffer_size {28};
    u8 tx_buffer[tx_buffer_size] {0};
    u8 rx_buffer[128] {0};

    /*
    for(int i=0;;i++) {
        for(size_t servo_index=0; servo_index<N; ++servo_index) {
            offset = servo_index * sizeof(SERVO_STATE) + 2;
	    pos = i % 1024;
            memcpy((char*)control_block + offset, &pos, 2);
        }
    } 
    */
    dev.filename = "/dev/ttyAMA1";
    dev.rate = B3000000;

    rc = uart_start(&dev, false);
    //TODO handle return code
    for (;;) {
	tx_buffer[0] = 0xFF;
	tx_buffer[1] = 0xFF;
	tx_buffer[2] = 0x01;
	tx_buffer[3] = 0x1C;
	index = 4;
        for(size_t servo_index=0; servo_index<N; ++servo_index) {
            offset = servo_index * sizeof(SERVO_STATE) + 2;
            memcpy(&tx_buffer[index], (char*)control_block + offset, 2);
	    index+=2;
	}
        uart_writen(&dev, (char *)tx_buffer, tx_buffer_size);
        read_data_len = uart_reads(&dev, (char *)rx_buffer, sizeof(rx_buffer));
    }
}

int main(int argc, char *argv[])
{
    struct sockaddr_un name;
    int down_flag = 0;
    size_t offset;
    size_t index;
    int ret;
    int pid;
    int connection_socket;
    int data_socket;
    int result;
    static u8 r_buffer[buffer_size];
    static u8 s_buffer[buffer_size];

    control_block = mmap(NULL, sizeof state, PROT_READ | PROT_WRITE,
                         MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    memcpy(control_block, state, sizeof(state));

    /* start UART protocol with ESP32 */
    pid = fork();
    if (pid == 0) {
	esp32_protocol(control_block);
    }

    /* Create local socket. */

    connection_socket = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (connection_socket == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    /*
     * For portability clear the whole structure, since some
     * implementations have additional (nonstandard) fields in
     * the structure.
     */

    memset(&name, 0, sizeof(name));

    /* Bind socket to socket name. */

    name.sun_family = AF_UNIX;
    strncpy(name.sun_path, SOCKET_NAME, sizeof(name.sun_path) - 1);
    ret = unlink(SOCKET_NAME);
    if (ret == -1 && errno != ENOENT) {
        printf("unlink %s", strerror(errno));
        perror("unlink");
        exit(EXIT_FAILURE);
    }

    ret = bind(connection_socket, (const struct sockaddr *) &name,
               sizeof(name));
    if (ret == -1) {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    /*
     * Prepare for accepting connections. The backlog size is set
     * to 20. So while one request is being processed other requests
     * can be waiting.
     */

    ret = listen(connection_socket, 20);
    if (ret == -1) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    /* This is the main loop for handling connections. */

    for (;;) {

        /* Wait for incoming connection. */

        data_socket = accept(connection_socket, NULL, NULL);
        if (data_socket == -1) {
            perror("accept");
            exit(EXIT_FAILURE);
        }

	pid = fork();
        if (pid == 0) {
    
            result = 0;
            for (;;) {
    
                /* Wait for next data packet. */
    
                ret = read(data_socket, r_buffer, sizeof(r_buffer));
                if (ret == -1) {
    		/* client terminated? */
    		break;
                }
    
                /* Handle commands. */
                index = 2;
    
    	    if(r_buffer[1] == INST_SETPOS && r_buffer[0] == buffer_size) {
                for(size_t servo_index=0; servo_index<N; ++servo_index) {
		    offset = servo_index * sizeof(SERVO_STATE) + 2;
    		    memcpy((char*)control_block + offset, &r_buffer[index], 2);
    		    index = index + 2;
    		}
    		s_buffer[0]= 2;
    		s_buffer[1]= INST_SETPOS;
    	    }
    
    	    if(r_buffer[1] == INST_GETPOS && r_buffer[0] == 2) {
    		s_buffer[0]= buffer_size;
    		s_buffer[1]= INST_GETPOS;
                for(size_t servo_index=0; servo_index<N; ++servo_index) {
		    offset = servo_index * sizeof(SERVO_STATE) + 2;
    		    memcpy(&s_buffer[index], (char*)control_block + offset, 2);
    		    index = index + 2;
    		}
    	    }
    
                /* Send result. */
        
                ret = write(data_socket, s_buffer, s_buffer[0]);
                if (ret == -1) {
    		/* client terminated? */
    		break;
                }
        
            }
        }
        else {
            close(data_socket);
            continue;
        }   
    }

    close(connection_socket);

    /* Unlink the socket. */

    unlink(SOCKET_NAME);

    exit(EXIT_SUCCESS);
}
