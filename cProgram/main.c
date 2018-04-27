#include <stdio.h>
#include <sys/un.h>
#include <sys/socket.h>

#define SOCKET_FILE "/tmp/myserver.sock"
#define BUF_SIZE    64 * 1024

int main() {
  struct sockaddr_un server_address = {AF_UNIX, SOCKET_FILE};

  int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
  if (sock <= 0) {
      perror("socket creation failed");
      return 1;
  }

  unlink(SOCKET_FILE);

  if (bind(sock, (const struct sockaddr *) &server_address, sizeof(server_address)) < 0) {
      perror("bind failed");
      close(sock);
      return 1;
  }

  for (;;) {
    struct sockaddr_un client_address;
    int i, numBytes, len = sizeof(struct sockaddr_un);
    char buf[BUF_SIZE];
    numBytes = recvfrom(sock, buf, BUF_SIZE, 0, (struct sockaddr *) &client_address, &len);
    if (numBytes == -1) puts("recvfrom failed");

    printf("Server received %d bytes from %s\n", numBytes, client_address.sun_path);

    for (i = 0; i < numBytes; i++)
      buf[i] = toupper((unsigned char) buf[i]);

    if (sendto(sock, buf, numBytes, 0, (struct sockaddr *) &client_address, len) != numBytes)
      puts("sendto failed");
  }

}