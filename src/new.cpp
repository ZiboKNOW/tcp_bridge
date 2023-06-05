#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 8080

using namespace std;

int main(int argc, char const *argv[]) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in servaddr, cliaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = htons(PORT);

    if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        perror("connection failed");
        exit(EXIT_FAILURE);
    }

    int matrix_size = 64 * 96 * 192;
    int* matrix = new int[matrix_size];

    // Fill the matrix with test data
    for (int i = 0; i < matrix_size; i++) {
        matrix[i] = i;
    }

    // Send the matrix over the network
    int bytes_sent = 0;
    while (bytes_sent < matrix_size * sizeof(int)) {
        int bytes_to_send = matrix_size * sizeof(int) - bytes_sent;
        int sent = send(sockfd, matrix + bytes_sent / sizeof(int), bytes_to_send, 0);
        if (sent < 0) {
            perror("send failed");
            exit(EXIT_FAILURE);
        }
        bytes_sent += sent;
    }

    delete[] matrix;
    close(sockfd);

    return 0;
}



#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 8080

using namespace std;

int main(int argc, char const *argv[]) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in servaddr, cliaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(sockfd, 1) < 0) {
        perror("listen failed");
        exit(EXIT_FAILURE);
    }

    int connfd = accept(sockfd,(struct sockaddr*)&cliaddr, (socklen_t*)&addrlen);
    if (connfd < 0) {
        perror("accept failed");
        exit(EXIT_FAILURE);
    }

    int matrix_size = 64 * 96 * 192;
    int* matrix = new int[matrix_size];

    // Receive the matrix from the network
    int bytes_received = 0;
    while (bytes_received < matrix_size * sizeof(int)) {
        int bytes_to_receive = matrix_size * sizeof(int) - bytes_received;
        int received = recv(connfd, matrix + bytes_received / sizeof(int), bytes_to_receive, 0);
        if (received < 0) {
            perror("receive failed");
            exit(EXIT_FAILURE);
        }
        bytes_received += received;
    }

    // Reassemble the matrix
    int*** three_dim_matrix = new int**[64];
    for (int i = 0; i < 64; i++) {
        three_dim_matrix[i] = new int*[96];
        for (int j = 0; j < 96; j++) {
            three_dim_matrix[i][j] = new int[192];
            for (int k = 0; k < 192; k++) {
                three_dim_matrix[i][j][k] = matrix[i * 96 * 192 + j * 192 + k];
            }
        }
    }

    // Do something with the reassembledmatrix

    // Free memory
    for (int i = 0; i < 64; i++) {
        for (int j = 0; j < 96; j++) {
            delete[] three_dim_matrix[i][j];
        }
        delete[] three_dim_matrix[i];
    }
    delete[] three_dim_matrix;
    delete[] matrix;
    close(sockfd);
    close(connfd);

    return 0;
}