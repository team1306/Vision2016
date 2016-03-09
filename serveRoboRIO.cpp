#include "serveRoboRIO.h"

#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>

#include <unistd.h>
#include <string.h>

#include <mutex>
#include <iostream>

void ServeRoboRIOUtil(std::mutex& dataMtx, visionData& data) {
  struct sockaddr_in servaddr;
  int sockfd, new_fd;

  // Create tcp/ip socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);

  //fcntl(sockfd, F_SETFL, O_NONBLOCK);

  bzero(&servaddr, sizeof(servaddr));
  
  // Tells the socket which port to listen on
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htons(INADDR_ANY);
  servaddr.sin_port = htons(5802);

  // Bind the socket to the port and listen for incoming connections
  bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
  std::cout << "Bound" << std::endl;
  // The number here is the number of connections allowed to be queued. We won't need more than one, but for some reason it doesn't work when it equals one
  while(1) {
    listen(sockfd, 10);

    // Wait for a connection and accept it
    new_fd = accept(sockfd, (struct sockaddr*) NULL, NULL);
    std::cout << "Accepted connection" << std::endl;
    char incoming [100];
    int s = 0;
    double dist;
    std::string message;

    // Communicate forever
    while(1) {
      bzero(incoming, 100);
      // Read incoming stream into incoming
      s = recv(new_fd, incoming, 100, 0);
      std::cout << "Received data" << std::endl;
      // If there is data and there wasn't an error (s = -1)
      if(s > 0) {
	// Convert incoming string to a double reprsenting the distance to the target
	std::cout << "Good message" << std::endl;
	/*lateralMtx.lock();
	lat = lateral;
	lateralMtx.unlock();*/
	double pitch, yaw, distance;
	dataMtx.lock();
	pitch = data.pitch;
	yaw = data.yaw;
	distance = data.distance;
	dataMtx.unlock();
    
	// Send the horizontal distance to the target to the RoboRIO
	message = std::to_string(pitch) + "," + std::to_string(yaw) + "," + std::to_string(distance) + "\n";
	send(new_fd, message.c_str(), message.size(), 0);
      }
      else {
	break;
      }
    }
    std::cout << "Shutting down" << std::endl;

    // If, for some reason, the loop terminates, close the sockets
    shutdown(new_fd, 2);
  }
  shutdown(sockfd, 2);
}
