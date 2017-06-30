#include<stdio.h> //printf
#include<string.h>    //strlen
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr

#include <ros/ros.h>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

int main(int argc , char *argv[])
{
  
  int port_nb = 8888;
  if (argc > 1) {
    port_nb = atoi(argv[1]);
  }
    int sock;
    struct sockaddr_in server;
    char message[1000] , server_reply[2000];
     
    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");
     
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    server.sin_family = AF_INET;
    server.sin_port = htons( port_nb );
 
    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }
     
    puts("Connected\n");
     
    //keep communicating with server
    while(1)
    {
        printf("Enter message : ");
        scanf("%s" , message);
  
        StringBuffer s;
        Writer<StringBuffer> writer(s);
        
        writer.StartObject();               // Between StartObject()/EndObject(), 
//         writer.Key("s");                // output a key,
//         writer.String("world");             // follow by a value.
//         writer.Key("t");
//         writer.Bool(true);
//         writer.Key("f");
//         writer.Bool(false);
//         writer.Key("n");
//         writer.Null();
        writer.Key("i");
        writer.Uint(123);
        writer.Key("pi");
        writer.Double(3.1416);
        writer.Key("a");
        writer.StartArray();                // Between StartArray()/EndArray(),
        for (unsigned i = 0; i < 4; i++)
            writer.Uint(i);                 // all values are elements of the array.
        writer.EndArray();
        writer.EndObject();

        // Copy from ptr to array.
        strcpy(message, s.GetString());

         
        //Send some data
        if( send(sock , message , strlen(message) , 0) < 0)
        {
            puts("Send failed");
            return 1;
        }
         
        //Receive a reply from the server
        if( recv(sock , server_reply , 2000 , 0) < 0)
        {
            puts("recv failed");
            break;
        }
         
        puts("Server reply :");
        puts(server_reply);
    }
     
    close(sock);
    return 0;
}