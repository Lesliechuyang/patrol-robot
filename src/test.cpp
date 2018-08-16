#include "ros/ros.h"
#include "patrol_robot/SendCommands.h"
#include <cstdlib>

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#define SERVER_PORT 5000
#define LENGTH_OF_LISTEN_QUEUE 20
#define BUFFER_SIZE 100

struct command_tmp {
    int id;
	float x;
	float y;
	float high;
	float camera_lr;//left and right
	float camera_ud;//up and down
};

int main(int argc, char **argv)
{
    //socket
    struct sockaddr_in server_addr;
    int server_socket;
    int opt = 1;

    bzero(&server_addr, sizeof(server_addr)); // 置字节字符串前n个字节为0，包括'\0'
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY); // 转小端,INADDR_ANY就是指定地址为0.0.0.0的地址
    server_addr.sin_port = htons(SERVER_PORT);  

    // 创建一个Socket
    server_socket = socket(PF_INET, SOCK_STREAM, 0);

    if (server_socket < 0)
    {
        printf("Create Socket Failed!\n");
        exit(1);
    }


    // bind a socket
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if(bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
    {
        printf("Server Bind Port: %d Failed!\n", SERVER_PORT);
        exit(1);
    }

    // 监听Socket
    if (listen(server_socket, LENGTH_OF_LISTEN_QUEUE))
    {
        printf("Server Listen Failed!\n");
        exit(1);
    }      

    //ros相关
    ros::init(argc, argv, "patrol_robot_client");

    ros::NodeHandle n;
    //ros::Rate r(1);
    ros::ServiceClient client = n.serviceClient<patrol_robot::SendCommands>("send_goals");
    patrol_robot::SendCommands srv;

    patrol_robot::Command command;//临时存放命令点
    std::vector<patrol_robot::Command> commands;

    while(n.ok()){
        struct sockaddr_in client_addr;
        int client_socket;
        socklen_t length;
        char Buffer[BUFFER_SIZE];

        // 连接客户端Socket
        length = sizeof(client_addr);
        client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &length);
        if (client_socket < 0)
        {
            printf("Server Accept Failed!\n");
            break;
        }

        // 从客户端接收数据
        while(1)
        {
            //接受command个数
            bzero(Buffer, BUFFER_SIZE);
            length = recv(client_socket, Buffer, BUFFER_SIZE, 0);

            if (length < 0)
            {
                printf("Server Recieve Data Failed!\n");
                break;
            }           
            
            if(length > 0){         
                //printf("num_s = %s\n", Buffer);
                int num = atoi(Buffer);
                //printf("num = %d\n", num);

                //接受成功，开始发送
                int ret = send(client_socket, Buffer, (int)strlen(Buffer), 0);
                //printf("ret = %d\n", ret);
                if(ret <= 0)
                {
                    printf("Send num failed\n");
                }
                else
                {
                    printf("Send num successful\n");
                    //发送成功，开始监听
                    for(int i = 0; i < num; i++)
                    {
                        struct command_tmp c;
                        length = recv(client_socket, (char*)&c, sizeof(command_tmp), 0);
                        if(length == sizeof(command_tmp))
                        {
                            command.goal.position.x = c.x;
                            command.goal.position.y = c.y;
                            command.elevator = c.high;
                            command.camera.x = c.camera_lr;
                            command.camera.y = c.camera_ud;
                            commands.push_back(command);
                        }
                        else
                        {
                            printf("The %dth command length error!\n", i + 1);
                            break;
                        }
                    }
                    //printf("commands.size() = %d\n", commands.size());
                    // for(int i = 0; i < commands.size(); i++){
                    //     printf("camera_ud = %f\n", commands[i].camera.y);
                    // }

                    if(commands.size() != num)
                    {
                        printf("Command number error, don't send service!\n");
                    }
                    else
                    {
                        //发送service
                        srv.request.commands.commands.resize(commands.size());//拷贝数据
                        for(unsigned int i = 0; i < commands.size(); ++i){
                            srv.request.commands.commands[i] = commands[i];
                        }
                        
                        if (client.call(srv))
                        {
                            if(srv.response.success)
                            {
                                ROS_INFO("Result: successful!");
                            }
                            else
                            {
                                ROS_INFO("Result: failed!");
                            }
                        }
                        else  
                        {
                            ROS_ERROR("Failed to call service");
                        }
                    }
                }
            }
            commands.clear();//清空vector
        }
        close(client_socket);
    }
    close(server_socket);
    return 0;
}