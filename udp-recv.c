/*
        udp-recv: a simple udp server
       
        Sniff, receive, and display UDP packets delivered to a specified port (within port.h)
        Biscuit device is sending to port 4120.

        usage:  ./udp-recv

        A modified version of Paul Krzyzanowski's demo-udp-03
        https://www.cs.rutgers.edu/~pxk/417/notes/sockets/demo-udp-03.html
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "port.h"

#define BUFSIZE 2048

int main(int argc, char **argv){

	struct sockaddr_in myaddr;	/* our address */
	struct sockaddr_in remaddr;	/* remote address */
	socklen_t addrlen = sizeof(remaddr);	/* length of addresses */
	int recvlen;			/* # bytes received */
	int fd;				/* our socket */
	unsigned char buf[BUFSIZE];	/* receive buffer */

	// create a UDP socket
        // this is what the biscuit will send UDP packets to

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return 0;
	}

	/* bind the socket to any valid IP address and a specific port (specified in port.h)*/

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(SERVICE_PORT);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		perror("bind failed");
		return 0;
	}

	/* now loop infinitely, waiting for data, receiving data, printing what we received */
	while(1) {
		printf("waiting on port %d\n", SERVICE_PORT);
		recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);

		printf("Received %d bytes\n", recvlen);
                printf("Received packet from %s:%d\n",
                   inet_ntoa(remaddr.sin_addr),ntohs(remaddr.sin_port));
		
                if (recvlen > 0) {
			buf[recvlen] = 0;
			printf("received message:\n\%s\n", buf);
		}
	}
}
