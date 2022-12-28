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

#include "mini_pupper_host_base.h"
#include "mini_pupper_protocol.h"

static void *control_block;

void esp32_protocol(void *control_block)
{
    struct UartDevice dev;
    dev.filename = "/dev/ttyAMA1";
    dev.rate = B3000000;
    int result = uart_start(&dev, false);
    printf("uart_start:%d\r\n",result);

    //TODO handle return code
    for (;;)
    {
        /*
         * Encode a CONTROL frame 
         */

        // Build parameters
        parameters_control_instruction_format parameters;

        // Copy 12 servo goal positions into the goal_position array of parameters
        for(auto & goal_position : parameters.goal_position)
        {
            goal_position = 512; // default hard-coded position (note : to be replaced by following line of codes)
        }
        /*
                for(size_t servo_index=0; servo_index<N; ++servo_index) 
                {
                    offset = servo_index * sizeof(SERVO_STATE) + 2;
                    memcpy(&tx_buffer[index], (char*)control_block + offset, 2);
                    index+=2;
                }
        */

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
            INST_CONTROL       // instruction
        };
        memcpy(tx_buffer+5,&parameters,sizeof(parameters_control_instruction_format));
        
        // Checksum
        tx_buffer[tx_buffer_size-1] = compute_checksum(tx_buffer);

        // Send
        result = uart_writen(&dev, (char *)tx_buffer, tx_buffer_size);
        printf("uart_writen:%d\r\n",result);

        // INSERT a 25ms delay
        // INSERT a 25ms delay
        // INSERT a 25ms delay


        /*
         * Decode a CONTROL ACK frame 
         */

        // Read buffer
        size_t const rx_buffer_size { 128 };
        u8 rx_buffer[rx_buffer_size] {0};
        
        // Read
        int read_length = uart_reads(&dev, (char*)rx_buffer, 4); 
        printf("uart_reads:%d\r\n",read_length);

        // waiting for a (full) header...
        if(read_length != 4)
        {
            // log
            printf("RX frame header truncated!");
            // flush RX FIFO
            //// HOW TO LINUX ?
            // next            
            continue;
        }

        // waiting for a valid frame header with my ID...
        bool const rx_header_check { 
                    (rx_buffer[0]==0xFF) 
                &&  (rx_buffer[1]==0xFF) 
                &&  (rx_buffer[2]==0x01) // my ID
                &&  (rx_buffer[3]<=(rx_buffer_size-4)) // keep the header in the rx buffer
        };  
        if(!rx_header_check) 
        {
            // log
            printf("RX frame error : header invalid!");
            // flush RX FIFO
            //// HOW TO LINUX ?
            // next
            continue;
        }

        // read payload length from frame header
        size_t const rx_payload_length {(size_t)rx_buffer[3]};

        // copy RX fifo into local buffer (L bytes : Payload + Checksum)
        read_length = uart_reads(&dev, (char*)(rx_buffer+4), rx_payload_length); 

        // waiting for a (full) payload...
        if(read_length != (int)rx_payload_length) 
        {
            // log
            printf("RX frame error : truncated payload [expected:%d, received:%d]!",rx_payload_length,read_length);
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
                    (rx_buffer[4]==INST_CONTROL) 
                &&  rx_checksum
        };  
        if(!rx_payload_checksum_check) 
        {
            // log
            printf("RX frame error : bad instruction [%d] or checksum [received:%d,expected:%d]!",rx_buffer[4],rx_buffer[rx_payload_length+4-1],expected_checksum);
            // flush RX FIFO
            //// HOW TO LINUX ?
            // next
            continue;
        }

        // decode parameters
        parameters_control_acknowledge_format ack_parameters;
        memcpy(&ack_parameters,rx_buffer+5,sizeof(parameters_control_acknowledge_format));

        // log
        printf("Present Position: %d %d %d %d %d %d %d %d %d %d %d %d",
            ack_parameters.present_position[0],ack_parameters.present_position[1],ack_parameters.present_position[2],
            ack_parameters.present_position[3],ack_parameters.present_position[4],ack_parameters.present_position[5],
            ack_parameters.present_position[6],ack_parameters.present_position[7],ack_parameters.present_position[8],
            ack_parameters.present_position[9],ack_parameters.present_position[10],ack_parameters.present_position[11]
        );
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
