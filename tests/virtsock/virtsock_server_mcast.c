/* server.c */
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
static int port = 6666;

int main(void)
{
	int socket_descriptor;
	struct sockaddr_in address;
	socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_descriptor == -1) {
		perror("socket()");
		exit(EXIT_FAILURE);
	}
	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr("224.0.0.1");
	address.sin_port = htons(port);

	if(fcntl(socket_descriptor, F_SETFL, fcntl(socket_descriptor, F_GETFL) | O_NONBLOCK) < 0) {
		perror("fcntl:SOCK_SET_NONBLOCK");
	}

	printf("Server ist bereit ...\n");
	/* Broadcasting beginnen */
	while (1) {
		char message[256];
		if (sendto(socket_descriptor, "broadcast test (hallo client)",
		                sizeof("broadcast test (hallo client)"), 0,
		                (struct sockaddr *)&address, sizeof(address))
		                < 0) {
			perror("sendto()");
			exit(EXIT_FAILURE);
		}
		recv(socket_descriptor, message, 256, 0);
		sleep(1);
	}
	return EXIT_SUCCESS;
}
