#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(),connect(),send() and recv() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;

#define TEMPLATE "6-{\"build\":\"%s\",\
                    \"mcuclk\":\"%ld Hz\",\
                    \"imei\":\"%s\",\
                    \"memory\":\"%d MB\",\
                    \"license\":\"%s\",\
                    \"add\":\"%s\",\
                    \"light\":%d,\
                    \"group\":%d,\
                    \"mode\":\"%s\"}\n"

// \"power\":\"%s\",
#define MAX 255
#define PORT 8889
#define SA struct sockaddr

pthread_t thread_Client;
pthread_t thread_Serial;

int cur_client = -1;

struct ThreadArgs
{
    int clntSock; /* Socket descriptor for client */
};

ThreadArgs *threadArgs;

typedef struct __attribute__((__packed__))
{
    char build[25];
    uint32_t mcu_clock;
    char IMEI[20];
    uint8_t rom_memory;
    char license[20];
    char add[80];
    uint8_t numlamp;
    uint8_t num_group;
    uint8_t mode_control;
} sys_info_t;

sys_info_t sys_info;

typedef struct __attribute__((packed))
{
    uint8_t idx;     //dia chi den hoac nhom . Den 1 den xx den. Nhom 1-10
    uint8_t percent; //0-100% do sang den
} manual_packet_t;

manual_packet_t manual_packet;

#define MSP_EX_GET_STARTUP 239     //ma yeu cau va nhan du lieu thong tin he thong
#define MSP_EX_SET_POWERSUPPLY 240 //ma dieu khien nguon
#define MSP_EX_SET_MODE 242        //ma set che do lam viec

#define MSP_EX_DIM_GROUP 243      //ma dim theo group
#define MSP_EX_DIM_ADD 244        //ma dim dia theo dia chi
#define MSP_EX_GET_INFO_ID 245    //Lay thong tin cua den nao do
#define MSP_EX_LOGCAT_MESSAGE 246 //Ma lenh hien thong bao string

typedef enum _serial_state
{
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
} _serial_state;
_serial_state c_state;

uint8_t checksum;
uint8_t indRX;
uint8_t inBuf[255];
uint8_t offset;
uint8_t dataSize;
uint8_t cmdMSP;

int fd;

//Truyen 1 byte
void serialize8(uint8_t a)
{
    serialPutchar(fd, a);
    checksum ^= a;
}

//Truyen 2 byte
void serialize16(int16_t a)
{
    static uint8_t t;
    t = a;
    serialPutchar(fd, t);
    checksum ^= t;
    t = (a >> 8) & 0xff;
    serialPutchar(fd, t);
    checksum ^= t;
}

//Truyen 4 byte
void serialize32(uint32_t a)
{
    static uint8_t t;
    t = a;
    serialPutchar(fd, t);
    checksum ^= t;
    t = a >> 8;
    serialPutchar(fd, t);
    checksum ^= t;
    t = a >> 16;
    serialPutchar(fd, t);
    checksum ^= t;
    t = a >> 24;
    serialPutchar(fd, t);
    checksum ^= t;
}

//
void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    checksum = 0;
    serialize8(s);
    serialize8(cmdMSP);
}

//
void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

//
void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

//
void tailSerialReply(void)
{
    serialize8(checksum);
}

//
void serializeNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(*c);
}
//
uint8_t read8(void)
{
    return inBuf[indRX++] & 0xff;
}
//
uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}
//
int16_t readint16(void) //
{
    int16_t temp = (inBuf[indRX++]);
    temp = temp + ((inBuf[indRX++]) << 8);
    return temp;
}
//
uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}

//
void send_struct(uint8_t cmd, uint8_t *cb, uint8_t siz)
{
    cmdMSP = cmd;
    headSerialReply(siz);
    while (siz--)
    {
        serialize8(*cb++);
    }
    tailSerialReply();
}

//
void send_byte(uint8_t cmd, uint8_t data)
{
    cmdMSP = cmd;
    headSerialReply(1);
    serialize8(data);
    tailSerialReply();
}

//
void readstruct(uint8_t *pt, uint8_t size)
{
    uint16_t i = 0;
    for (i = 0; i < size; i++)
    {
        *pt = inBuf[indRX++];
        pt++;
    }
}

char logcat_buff[255];

// Co` ngat
bool flag_info = false;
bool flag_message = false;

void get_data_finish(void)
{
    switch (cmdMSP)
    {
    case MSP_EX_GET_STARTUP:
        readstruct((uint8_t *)&sys_info, sizeof(sys_info_t));
        flag_info = true;
        break;
    case MSP_EX_LOGCAT_MESSAGE:
        readstruct((uint8_t *)&logcat_buff, sizeof(logcat_buff));
        flag_message = true;
    default:
        break;
    }
}

void serial_get_buffer(void)
{
    uint8_t c = 0;
    if (serialDataAvail(fd))
    {
        c = (uint8_t)(serialGetchar(fd));
        // cout << "This is check: " << c <<endl;
        if (c_state == IDLE)
        {
            c_state = (c == '$') ? HEADER_START : IDLE;
            if (c_state == IDLE)
            {
            }
        }
        else if (c_state == HEADER_START)
        {
            c_state = (c == 'M') ? HEADER_M : IDLE;
        }
        else if (c_state == HEADER_M)
        {
            c_state = (c == '>') ? HEADER_ARROW : IDLE;
        }
        else if (c_state == HEADER_ARROW)
        {
            if (c > 255)
            {
                c_state = IDLE;
            }
            else
            {
                dataSize = c;
                offset = 0;
                checksum = 0;
                indRX = 0;
                checksum ^= c;
                c_state = HEADER_SIZE;
            }
        }
        else if (c_state == HEADER_SIZE)
        {
            cmdMSP = c;
            checksum ^= c;
            c_state = HEADER_CMD;
        }
        else if (c_state == HEADER_CMD && offset < dataSize)
        {
            checksum ^= c;
            inBuf[offset++] = c;
        }
        else if (c_state == HEADER_CMD && offset >= dataSize)
        {
            if (checksum == c)
            {
                get_data_finish();
            }
            c_state = IDLE;
        }
    }
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

// Convert data to string

char *get_json_update(sys_info_t *info)
{
    if (info == NULL)
    {
        return NULL;
    }
    char *ret = NULL;
    asprintf(&ret, TEMPLATE,
             info->build,
             info->mcu_clock,
             info->IMEI,
             info->rom_memory,
             info->license,
             info->add,
             info->numlamp,
             info->num_group,
             // info->mode_control == 1 ? "manual" : "auto");
             // (player % 2 == 1) ? 1 : ((player % 2 == 0) ? 2 : 3);
             (info->mode_control == 1) ? "off" : (info->mode_control == 3) ? "manual" : "auto");
    return ret;
}
// Function designed for chat between client and server.

void *func(void *threadArgs)
{
    char buff[MAX];
    Json::Value root;
    Json::Reader reader;
    int len;
    int clientSock;
    // infinite loop for chat
    clientSock = ((struct ThreadArgs *)threadArgs)->clntSock;
    while (true)
    {
        len = read(clientSock, buff, sizeof(buff));
        if (len == 0)
        {
            printf("Socket disconnected\n");
            close(clientSock);
            break;
        }
        buff[len] = '\0';
        cout << "received buffer " << buff << endl;
        if (!strncmp(buff, "PING", 4))
        {
            write(clientSock, "PONG\n", 5);
        }

        if (!strncmp(buff, "6", 1))
        {

            send_byte(MSP_EX_GET_STARTUP, 1);
        }
        if (!strncmp(buff, "4-manual", 8))
        {
            send_byte(MSP_EX_SET_MODE, 1);
        }
        if (!strncmp(buff, "4-free", 5))
        {
        // cout << buff << endl;
            send_byte(MSP_EX_SET_MODE,0);
        }
        if (!strncmp(buff, "4-auto", 6))
        {
            // cout << buff << endl;
            send_byte(MSP_EX_SET_MODE,2);
        }
        if (!strncmp(buff, "3-on", 4))
        {
        //     cout << buff << endl;
            send_byte(MSP_EX_SET_POWERSUPPLY,1);
        }

        if (!strncmp(buff, "3-off", 5))
        {
        //     cout << buff << endl;
            send_byte(MSP_EX_SET_POWERSUPPLY,0);
        }

        if (!strncmp(buff, "1", 1))
        {
            reader.parse(buff + 2, root);
            cout << "light: " << root["light"].asUInt()<<"\t" << "dim: " << root["dim"].asUInt() << endl;
            manual_packet.idx = (uint8_t)root["light"].asUInt();
            manual_packet.percent = (uint8_t)root["dim"].asUInt();
            cout << "light: " << manual_packet.idx<<"\t" << "dim: " << manual_packet.percent << endl;
            send_struct(MSP_EX_DIM_ADD,(uint8_t *)&manual_packet,sizeof(manual_packet));
        }


        if (!strncmp(buff, "2", 1))
        {
            // 2-{"group":1,"dim":0}

            char *temp;
            cout << buff <<endl;
            temp = strtok((char *)&buff,":");

            temp = strtok(NULL,",");
            cout << "idx" << temp << endl;
            manual_packet.idx  = (uint8_t)atoi(temp);
            temp = strtok(NULL,":");
            temp = strtok(NULL,"}");
            cout << "percent" << temp << endl;
            manual_packet.percent = (uint8_t)atoi(temp);
            // reader.parse(buff+2, root);
            // cout << "group: " << root["group"].asString()<< "\t" << "dim: " << root["dim"].asString() << endl;
            // manual_packet.idx = root["group"].asUInt();
            // manual_packet.percent = root["dim"].asUInt();
            cout << "group: " << manual_packet.idx<< "\t" << "dim" << manual_packet.percent << endl;

            send_struct(MSP_EX_DIM_GROUP,(uint8_t *)&manual_packet,sizeof(manual_packet));
        }

        if (!strncmp(buff, "5", 1))
        {
            reader.parse(buff+2, root);
            cout << "light: " << root["light"].asString()<<endl;
            send_byte(MSP_EX_GET_INFO_ID,(uint8_t)root["light"].asUInt());
        }
    }
}


// if (!strncmp(buff, "4-auto", 6))
// {
//     cout << buff << endl;
//     send_byte(MSP_EX_SET_MODE,2);
//     while(1)
//     {
//         serial_get_buffer();
//         // cout << "get data" <<endl;
//         if (flag_message)
//         {
//             flag_message =false;
//             cout << logcat_buff << endl;
//             write(sockfd,&logcat_buff ,strlen(logcat_buff));
//             printf("Update complete\n");
//             break;
//         }
//     }
// }
// if (!strncmp(buff, "3-on", 4))
// {
//     cout << buff << endl;
//     send_byte(MSP_EX_SET_POWERSUPPLY,1);
//     while(1)
//     {
//         serial_get_buffer();
//         if (flag_message)
//         {
//             flag_message = false;
//             write(sockfd,&logcat_buff ,strlen(logcat_buff));
//             printf("Update complete\n");
//             break;
//         }
//     }
// }
// if (!strncmp(buff, "3-off", 5))
// {
//     cout << buff << endl;
//     send_byte(MSP_EX_SET_POWERSUPPLY,0);
//     while(1)
//     {
//         serial_get_buffer();
//         if (flag_message)
//         {
//             flag_message = false;
//             write(sockfd,&logcat_buff ,strlen(logcat_buff));
//             printf("Update complete\n");
//             break;
//         }
//     }
// }
// if (!strncmp(buff, "1", 1))
// {
//     reader.parse(buff + 2, root);
//     cout << "light: " << root["light"].asUInt()<<"\t" << "dim: " << root["dim"].asUInt() << endl;
//     manual_packet.idx = (uint8_t)root["light"].asUInt();
//     manual_packet.percent = (uint8_t)root["dim"].asUInt();
//     cout << "light: " << manual_packet.idx<<"\t" << "dim: " << manual_packet.percent << endl;
//     send_struct(MSP_EX_DIM_ADD,(uint8_t *)&manual_packet,sizeof(manual_packet));
//     while(1)
//     {
//         serial_get_buffer();
//         if (flag_message)
//         {
//             flag_message = false;
//             write(sockfd,&logcat_buff, strlen(logcat_buff));
//             printf("Update complete");
//             break;
//         }
//     }
// }

//  if (!strncmp(buff, "2", 1))
// {
//     // 2-{"group":1,"dim":0}

//     char *temp;
//     cout << buff <<endl;
//     temp = strtok((char *)&buff,":");

//     temp = strtok(NULL,",");
//     cout << "idx" << temp << endl;
//     manual_packet.idx  = (uint8_t)atoi(temp);
//     temp = strtok(NULL,":");
//     temp = strtok(NULL,"}");
//     cout << "percent" << temp << endl;
//     manual_packet.percent = (uint8_t)atoi(temp);
//     // reader.parse(buff+2, root);
//     // cout << "group: " << root["group"].asString()<< "\t" << "dim: " << root["dim"].asString() << endl;
//     // manual_packet.idx = root["group"].asUInt();
//     // manual_packet.percent = root["dim"].asUInt();
//     cout << "group: " << manual_packet.idx<< "\t" << "dim" << manual_packet.percent << endl;

//     send_struct(MSP_EX_DIM_GROUP,(uint8_t *)&manual_packet,sizeof(manual_packet));
//     while(1)
//     {
//         serial_get_buffer();
//         if (flag_message)
//         {
//             flag_message = false;
//             write(sockfd,&logcat_buff, strlen(logcat_buff));
//             printf("Update complete");
//             break;
//         }
//     }
// }

// if (!strncmp(buff, "5", 1))
// {
//     reader.parse(buff+2, root);
//     cout << "light: " << root["light"].asString()<<endl;
//     send_byte(MSP_EX_GET_INFO_ID,(uint8_t)root["light"].asUInt());
//     while(1)
//     {
//         serial_get_buffer();
//         if (flag_message)
//         {
//             flag_message = false;
//             write(sockfd,&logcat_buff, strlen(logcat_buff));
//             printf("Update complete");
//             break;
//         }
//     }
// }

// if (strncmp("exit", buff, 4) == 0)
// {
//     printf("Server Exit...\n");
//     break;
// }
//     }
// }

void *Serial(void *threadArgs)
{
    while (1)
    {
        serial_get_buffer();
        // cout << "Hello from thread serial" << endl;
        if (flag_info)
        {
            char *data = get_json_update(&sys_info);
            if (data)
            {
                if (cur_client >= 0) {
                    write(cur_client, data, strlen(data));
                    printf("Update complete\n");
                } else {
                    printf("client not connect\n");
                }
                free(data);
                flag_info = false;
            }
        }
        if (flag_message)
        {
            flag_message = false;
            cout << logcat_buff << endl;
            if (cur_client >= 0) {
                write(cur_client, &logcat_buff, strlen(logcat_buff));
                printf("Update complete\n");
            } else {
                printf("client not connect");
            }
            printf("Update complete\n");
        }
    }
}

int main()
{

    int sockfd, connfd;
    unsigned int len;
    struct sockaddr_in servaddr, cli;
    if ((fd = serialOpen("/dev/ttyUSB0", 115200)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        exit(-1);
    }

    if (pthread_create(&thread_Serial, NULL, Serial, NULL) != 0)
        error("thread_create() failed");

    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
        printf("socket creation failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully created..\n");
    bzero(&servaddr, sizeof(servaddr));

    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);

    // Binding newly created socket to given IP and verification
    if ((bind(sockfd, (SA *)&servaddr, sizeof(servaddr))) != 0)
    {
        printf("socket bind failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully binded..\n");

    // Now server is ready to listen and verification
    if ((listen(sockfd, 5)) != 0)
    {
        printf("Listen failed...\n");
        exit(0);
    }
    else
        printf("Server listening..\n");
    len = sizeof(cli);

    // Accept the data packet from client and verification
    // connfd = accept(sockfd, (SA*)&cli, &len);
    // if (connfd < 0) {
    //     printf("server acccept failed...\n");
    //     exit(0);
    // }
    // else
    //     printf("server acccept the client...\n");
    while (true)
    {
        connfd = accept(sockfd, (SA *)&cli, &len);
        if (connfd < 0)
        {
            printf("server acccept failed...\n");
            exit(0);
        }
        else
            printf("server acccept the client...\n");

        getpeername(connfd, (struct sockaddr *)&cli, &len);

        /* Create separate memory for client argument */
        if ((threadArgs = (struct ThreadArgs *)malloc(sizeof(struct ThreadArgs))) == NULL)
            error("malloc() failed");

        threadArgs->clntSock = connfd;
        cur_client = connfd;

        if (pthread_create(&thread_Client, NULL, func, (void *)threadArgs) != 0)
            error("pthread_create() failed");
        pthread_join(thread_Client, NULL);
        cur_client = -1;
    }

    // Function for chatting between client and server

    // After chatting close the socket
    // close(sockfd);
    return 0;
}
// install json lib: sudo apt-get install libjsoncpp-dev
// build g++ -ljsoncpp -lpthread TCPserver.cpp -o server -lwiringPi