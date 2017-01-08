/* uds_client.c */
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define BUF 1024
#define UDS_FILE "/tmp/osmocom_l2"
int main(int argc, char **argv) {
	int create_socket;
	char *buffer = malloc(BUF);
	struct sockaddr_un address;
	int size;
	if ((create_socket = socket(PF_LOCAL, SOCK_STREAM, 0)) > 0)
		printf("Socket wurde angelegt\n");
	address.sun_family = AF_LOCAL;
	strcpy(address.sun_path, UDS_FILE);
	if (connect(create_socket, (struct sockaddr *) &address, sizeof(address))
			== 0)
		printf("Verbindung mit dem Server hergestellt\n");
	while (strcmp(buffer, "quit\n") != 0) {
		printf("Nachricht zum Versenden: ");
		fgets(buffer, BUF, stdin);
		send(create_socket, buffer, strlen(buffer), 0);
	}
	close(create_socket);
	return EXIT_SUCCESS;
}
