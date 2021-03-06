#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

class SendData{

public:

  SendData(){};
  ~SendData(){};

  void* Sender(){


    /*********************************************
   format data to send
    *******************************************/
    char senddata[1000];
    char recvdata[20];
    int n;

    sprintf(senddata,"1,table=%scolumn=%svalue=%s",databaseName,columnType,values);

    printf("%s\n",senddata);

    struct sockaddr_in server;
    int sock;
    char deststr[80] = "db1.ertl.jp";
    unsigned int **addrptr;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      perror("socket");
      return;
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(5555); // HTTP port is 80 

    server.sin_addr.s_addr = inet_addr(deststr);
    if (server.sin_addr.s_addr == 0xffffffff) {
      struct hostent *host;

      host = gethostbyname(deststr);
      if (host == NULL) {
        if (h_errno == HOST_NOT_FOUND) {
          // h_errno is defined in extern 
          printf("host not found : %s\n", deststr);
        } else {

          printf("%s : %s\n", hstrerror(h_errno), deststr);
        }
        return;
      }

      addrptr = (unsigned int **)host->h_addr_list;

      while (*addrptr != NULL) {
        server.sin_addr.s_addr = *(*addrptr);

        // if connect() is succeed , exit loop 
        if (connect(sock,
                    (struct sockaddr *)&server,
                    sizeof(server)) == 0) {
          break;
        }

        addrptr++;
        // if connect is failed, try next address
      }

      // the case of connection failed 
      if (*addrptr == NULL) {
        perror("connect");
        return;
      }
    } else {
      if (connect(sock,
                  (struct sockaddr *)&server, sizeof(server)) != 0) {
        perror("connect");
        return;
      }
    }

    n = send(sock, senddata, (int)strlen(senddata),0);
    if (n < 0) {
      perror("write");
      return;
    }
    
    memset(recvdata, 0, sizeof(recvdata));
    n = recv(sock, recvdata, sizeof(recvdata),0);
    if (n < 0) {
      perror("read");
      return;
    }

    memset(senddata, 0, sizeof(senddata));
    memset(recvdata, 0, sizeof(recvdata));
    snprintf(senddata, sizeof(senddata),"0,exit\n");

    n = send(sock,senddata,(int)strlen(senddata),0);
    n = recv(sock,recvdata,sizeof(recvdata),0);

    printf("receive data:%s\n",recvdata);

    close(sock);


  }


  void setData(char *val){
    values = val;
  }

  void setDBName(char *db){
    databaseName = db;
  }

  void setColumnType(char *cl){
    columnType = cl;
  }


private:

  char *values;
  char *databaseName;
  char *columnType;

};
