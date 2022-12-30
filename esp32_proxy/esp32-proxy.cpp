#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "connection.h"

#include "mini_pupper_host_base.h"
#include "mini_pupper_protocol.h"

static void *control_block;

void esp32_protocol(void *control_block)
{
    static const char* filename = "/dev/ttyAMA1";
    int fd;
    int rc;
    int offset;
    struct termios *tty;
    bool print_debug;
    print_debug = false;

    fd = open(filename, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("%s: failed to open UART device\n", __func__);
        exit(EXIT_FAILURE);
    }
    tty = (termios *)malloc(sizeof(*tty));
    if (!tty) {
        printf("%s: failed to allocate UART TTY instance\n", __func__);
        exit(EXIT_FAILURE);
    }

    memset(tty, 0, sizeof(*tty));
    tty->c_iflag |=  IGNPAR;
    tty->c_cflag =  CS8 | CREAD | B3000000;
    tty->c_cc[VTIME] = 10; // Set timeout of 1.0 seconds
    tty->c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    rc = tcsetattr(fd, TCSANOW, tty);
    if (rc) {
        printf("%s: failed to set attributes\r\n", __func__);
        exit(EXIT_FAILURE);
    }

    for (;;)
    {
        /*
         * Encode a CONTROL frame 
         */

        // Build parameters
        parameters_control_instruction_format parameters;

        // Copy 12 servo goal positions into the goal_position array of parameters
        for(size_t servo_index=0; servo_index<N; ++servo_index) 
        {
            offset = servo_index * sizeof(SERVO_STATE) + 2;
            memcpy(&parameters.goal_position[servo_index], (char*)control_block + offset, 2);
        }

        // Compute the size of the payload (parameters length + 2)
        size_t const tx_payload_length { sizeof(parameters_control_instruction_format) + 2 };

        // Compute the size of the frame
        size_t const tx_buffer_size { tx_payload_length + 4 };

        // Build the frame
        u8 tx_buffer[tx_buffer_size] {
            0xFF,               // header
            0xFF,               // header
            0x01,               // default ID
            tx_payload_length,  // length
            INST_CONTROL        // instruction
        };
        memcpy(tx_buffer+5,&parameters,sizeof(parameters_control_instruction_format));
        
        // Checksum
        tx_buffer[tx_buffer_size-1] = compute_checksum(tx_buffer);

        // Send
        int result = write(fd, (char *)tx_buffer, tx_buffer_size);
        if (print_debug)
	{
	    printf("uart writen:%d\n",result);
	}

        // Read buffer
	// If we do not get data within 1 second we assume ESP32 stopped sending and we send again
        size_t const rx_buffer_size { 128 };
        u8 rx_buffer[rx_buffer_size] {0};
	int read_length;
	int expected_lenght;
	expected_lenght = 4 + 1 + 24 + 24 + 1; // header + status + pos + load + chksum
	offset = 0;
	do {
            read_length = read(fd, (char*)rx_buffer + offset, expected_lenght); 
	    if(!read_length)
	    {
		// time out
		break;
	    }
	    expected_lenght -= read_length;
	    offset += read_length;
            if (print_debug)
            {
                printf("uart read:%d, waiting for %d\n",read_length, expected_lenght);
            }
	} while (expected_lenght > 0);

	if(expected_lenght)
	{
	    // time out
	    continue;
	}


        /*
         * Decode a CONTROL ACK frame 
         */

        bool const rx_header_check { 
                    (rx_buffer[0]==0xFF) 
                &&  (rx_buffer[1]==0xFF) 
                &&  (rx_buffer[2]==0x01) // my ID
                &&  (rx_buffer[3]<=(rx_buffer_size-4)) // keep the header in the rx buffer
        };  
        if(!rx_header_check) 
        {
            // log
            if (print_debug)
	    {
                printf("RX frame error : header invalid!\n");
	    }
            // flush RX FIFO
            //// HOW TO LINUX ?
            // next
            continue;
        }

        // checksum
        u8 expected_checksum {0};
        bool const rx_checksum {checksum(rx_buffer,expected_checksum)};

        // waiting for a valid instruction and checksum...
        bool const rx_payload_checksum_check { 
                    (rx_buffer[4]==0x00) 
                &&  rx_checksum
        };  
        if(!rx_payload_checksum_check) 
        {
            // log
            if (print_debug)
	    {
                printf("RX frame error : bad instruction [%d] or checksum [received:%d,expected:%d]!\n",rx_buffer[4],rx_buffer[rx_buffer[3]+3],expected_checksum);
	    }
            // flush RX FIFO
            //// HOW TO LINUX ?
            // next
            continue;
        }

        // decode parameters
        parameters_control_acknowledge_format ack_parameters;
        memcpy(&ack_parameters,rx_buffer+5,sizeof(parameters_control_acknowledge_format));
        for(size_t servo_index=0; servo_index<N; ++servo_index) 
        {
            offset = servo_index * sizeof(SERVO_STATE) + 4;
            memcpy((char*)control_block + offset, &ack_parameters.present_position[servo_index], 2);
            memcpy((char*)control_block + offset + 4, &ack_parameters.present_load[servo_index], 2);
        }

        // log
        if (print_debug)
	{
            printf("Present Position: %d %d %d %d %d %d %d %d %d %d %d %d\n",
            ack_parameters.present_position[0],ack_parameters.present_position[1],ack_parameters.present_position[2],
            ack_parameters.present_position[3],ack_parameters.present_position[4],ack_parameters.present_position[5],
            ack_parameters.present_position[6],ack_parameters.present_position[7],ack_parameters.present_position[8],
            ack_parameters.present_position[9],ack_parameters.present_position[10],ack_parameters.present_position[11]
            );
            printf("            Load: %d %d %d %d %d %d %d %d %d %d %d %d\n",
            ack_parameters.present_load[0],ack_parameters.present_load[1],ack_parameters.present_load[2],
            ack_parameters.present_load[3],ack_parameters.present_load[4],ack_parameters.present_load[5],
            ack_parameters.present_load[6],ack_parameters.present_load[7],ack_parameters.present_load[8],
            ack_parameters.present_load[9],ack_parameters.present_load[10],ack_parameters.present_load[11]
            );
	}
    }
}

int main(int argc, char *argv[])
{
    struct sockaddr_un name;
    size_t offset;
    size_t index;
    int ret;
    int pid;
    int connection_socket;
    int data_socket;
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
		    offset = servo_index * sizeof(SERVO_STATE) + 4;
    		    memcpy(&s_buffer[index], (char*)control_block + offset, 2);
    		    index = index + 2;
    		}
    	    }
    
    	    if(r_buffer[1] == INST_GETLOAD && r_buffer[0] == 2) {
    		s_buffer[0]= buffer_size;
    		s_buffer[1]= INST_GETLOAD;
                for(size_t servo_index=0; servo_index<N; ++servo_index) {
		    offset = servo_index * sizeof(SERVO_STATE) + 8;
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
