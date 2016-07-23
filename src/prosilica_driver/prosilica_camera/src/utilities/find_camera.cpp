/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/console.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
                          
int receive_socket()
{
  int insock;
  struct sockaddr_in recvaddr;
  
  if ((insock = socket(PF_INET,SOCK_DGRAM,0)) == -1)
  {
    ROS_ERROR("Error opening input socket.");
    return -1;
  }
  
  // Address to receive at.
  memset(&recvaddr, 0, sizeof(recvaddr));
  recvaddr.sin_family = AF_INET;
  recvaddr.sin_addr.s_addr = INADDR_BROADCAST;
  recvaddr.sin_port = htons(1234);
  
  if (bind(insock, (struct sockaddr*) &recvaddr, sizeof recvaddr) == -1)
  {
    ROS_ERROR("Can't bind to socket to receive.");
    close(insock);
    return -1;
  }

  return insock;
}

int send_query(struct ifaddrs *iface, int srcport)
{
  int outsock;
  int port = 3956;
  int broadcast=1;
  struct sockaddr_in sendaddr;
  struct sockaddr_in recvaddr;

  if ((outsock = socket(PF_INET,SOCK_DGRAM,0)) == -1)
  {
    ROS_ERROR("Error opening output socket.");
    return -1;
  }

  // Address to receive at.
  memset(&sendaddr, 0, sizeof(sendaddr));
  sendaddr.sin_family = AF_INET;
  sendaddr.sin_addr.s_addr = ((sockaddr_in *) iface->ifa_addr)->sin_addr.s_addr;
  sendaddr.sin_port = htons(srcport);

  if (bind(outsock, (struct sockaddr*) &sendaddr, sizeof sendaddr) == -1)
  {
    ROS_ERROR("Can't bind to socket on interface %s.", iface->ifa_name);
    close(outsock);
    return -1;
  }

  // Send a broadcast.
  if ((setsockopt(outsock,SOL_SOCKET,SO_BROADCAST, &broadcast,sizeof broadcast)) == -1)
  {
    ROS_ERROR("Can't set broadcast flag on interface %s.", iface->ifa_name);
    close(outsock);
    return -1;
  }

  memset(&recvaddr, 0, sizeof(recvaddr));
  recvaddr.sin_family = AF_INET;
  recvaddr.sin_port = htons(port);
  recvaddr.sin_addr.s_addr = INADDR_BROADCAST;

  char outpkt[] = { 0x42, 0x11, 0x00, 0x02, 0x00, 0x00, 0x06, 0x26 };

  if (sendto(outsock, outpkt, sizeof(outpkt) , 0, (struct sockaddr *)&recvaddr, sizeof(recvaddr)) != sizeof(outpkt))
  {
    ROS_ERROR("Failed sending packet on interface %s.", iface->ifa_name);
    close(outsock);
    return -1;
  }

  close(outsock);
  return 0;
}

void get_response(int fd)
{
  ROS_INFO("Got a response on socket %i.", fd);
}

int collect_replies(fd_set *master_set, int maxfd)
{
  struct timeval tv;
  int retval;
  fd_set tmpset;

  tv.tv_sec = 1;
  tv.tv_usec = 0;

  while (1)
  {
    memcpy(&tmpset, master_set, sizeof(tmpset));
    retval = select(maxfd + 1, &tmpset, NULL, NULL, &tv);

    if (retval < 0 && errno != EINTR)
    {
      ROS_ERROR("Select exited with an error code.");
      return -1;
    }

    if (!retval)
      return 0;

    ROS_INFO("Got packet %i.", retval);
    for (int i = 0; i <= maxfd && retval > 0; i++)
      if (FD_ISSET(i, &tmpset))
      {
        get_response(i);
        retval--;
      }
  }
}
     
int get_local_port(int fd)
{
  struct sockaddr localPort;
  socklen_t localPortLen;

  if (getsockname(fd, &localPort, &localPortLen) == -1)
  {
    ROS_ERROR("Unable to get local port for socket.");
    return -1;
  }

  return ntohs(((struct sockaddr_in *)&localPort)->sin_port);
}

int main()
{
  struct ifaddrs *ifaces = NULL;
  fd_set fdset;
  int maxfd = 0;

  if (getifaddrs(&ifaces))
  {
    ROS_ERROR("Couldn't get interface list.");
    return 1;
  }

  FD_ZERO(&fdset);
  maxfd = receive_socket();
  if (maxfd < 0)
    return 1;
  FD_SET(maxfd, &fdset);
  
  int port = get_local_port(maxfd);

  for (struct ifaddrs *curif = ifaces; curif; curif = curif->ifa_next)
  {
    if (curif->ifa_addr && curif->ifa_addr->sa_family == AF_INET)
      send_query(curif, port);
  }
                                       
  freeifaddrs(ifaces); 

  return collect_replies(&fdset, maxfd) != 0;
}
