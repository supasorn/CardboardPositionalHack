/*
** client.c -- a stream socket client demo
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <linux/input.h>
#include <fcntl.h>
#include <X11/Xlib.h>

#include <arpa/inet.h>

#define PORT "38300" // the port client will be connecting to

#define MAXDATASIZE 100 // max number of bytes we can get at once
#define MOUSEFILE "/dev/input/event8"
// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

void mouseLoop(int sockfd) {
  int fd;
  struct input_event ie;
  Display *dpy;
  Window root, child;
  int rootX, rootY, winX, winY;
  unsigned int mask;

  printf("a\n");
  dpy = XOpenDisplay(NULL);
  XQueryPointer(dpy,DefaultRootWindow(dpy),&root,&child,
      &rootX,&rootY,&winX,&winY,&mask); 

  printf("b\n");
  if((fd = open(MOUSEFILE, O_RDONLY)) == -1) {
    perror("opening device");
    exit(EXIT_FAILURE);
  }

  printf("c\n");
  while(read(fd, &ie, sizeof(struct input_event))) {
    if (ie.type == 2) {
      if (ie.code == 0) { rootX += ie.value; }
      else if (ie.code == 1) { rootY += ie.value; }
      printf("time%ld.%06ld\tx %d\ty %d\n", 
          ie.time.tv_sec, ie.time.tv_usec, rootX, rootY);

      char tmp[30];
      sprintf(tmp, "%d,%d ", rootX,rootY);
      if (send(sockfd, tmp, strlen(tmp), 0) == -1)
        perror("send");

    } else if (ie.type == 1) {
      if (ie.code == 272 ) { 
        printf("Mouse button ");
        if (ie.value == 0)  
          printf("released!!\n");
        if (ie.value == 1)  
          printf("pressed!!\n");
      } else {
        printf("time %ld.%06ld\ttype %d\tcode %d\tvalue %d\n",
            ie.time.tv_sec, ie.time.tv_usec, ie.type, ie.code, ie.value);
      }
    }
  }
  printf("d\n");
}


int main(int argc, char *argv[]) {
  int sockfd, numbytes;
  char buf[MAXDATASIZE];
  struct addrinfo hints, *servinfo, *p;
  int rv;
  char s[INET6_ADDRSTRLEN];

  if (argc != 2) {
    fprintf(stderr,"usage: client hostname\n");
    exit(1);
  }

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  if ((rv = getaddrinfo(argv[1], PORT, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
  }

  // loop through all the results and connect to the first we can
  for(p = servinfo; p != NULL; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, p->ai_socktype,
            p->ai_protocol)) == -1) {
      perror("client: socket");
      continue;
    }

    if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      perror("client: connect");
      continue;
    }

    break;
  }

  if (p == NULL) {
    fprintf(stderr, "client: failed to connect\n");
    return 2;
  }

  inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
      s, sizeof s);
  printf("client: connecting to %s\n", s);

  freeaddrinfo(servinfo); // all done with this structure

  mouseLoop(sockfd);
  /*
  for (int i = 0; i < 10; i++) {
    char tmp[32];
    sprintf(tmp, "%dHello", i);
    printf("Sending %s\n", tmp);
    if (send(sockfd, tmp, 6, 0) == -1)
      perror("send");
  }
  printf("here\n");*/
  return 0;
  if ((numbytes = recv(sockfd, buf, MAXDATASIZE-1, 0)) == -1) {
    perror("recv");
    exit(1);
  }

  buf[numbytes] = '\0';

  printf("client: received '%s'\n",buf);

  close(sockfd);

  return 0;
}
