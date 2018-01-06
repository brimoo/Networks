/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#define INFINITY      9999
#define MAX           20
#define TIMEOUT       140000

typedef struct Neighbor{         // Use a struct to hold neighbor src and age(to account for drop outs)

    uint16_t srcNode;
    uint16_t Age;

} Neighbor;

#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"
#include "includes/socket.h"
#include "includes/TCPpacket.h"

module Node{
   uses interface Boot;

   uses interface SplitControl as AMControl;

   uses interface Receive;

   uses interface Random as Random;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;
   
   uses interface List<Neighbor> as NeighborList;

   uses interface Hashmap<uint16_t> as RoutingTable;

   uses interface Timer<TMilli> as neighborTimer;

   uses interface Timer<TMilli> as transmitTimer;

   uses interface List<pack> as PacketList; 

   uses interface List<socket_t> as SocketList; 
}

implementation{
   pack sendPackage;
   pack inFlight;
   uint16_t seqNum = 0;           // Increment the seqNum for each new pakcet

   uint16_t LSTable[MAX][MAX];      // Store each node's view of the topology
   
   // Project 1 Prototypes

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   void findNeighbors();
   void addPack(pack Package);
   bool checkForPack(pack *Package);
   bool isNeighbor(uint16_t src);

   // End Project 1 Prototypes

   // Project 2 Prototypes
  
   void initLSTable(); 
   void sendLSPacket();
   void updateLSTable(uint8_t * payload, uint16_t source);
   void printLSTable();
   void dijkstra();
   uint16_t minDist(uint16_t dist[], bool sptSet[]);
   void printRoutingTable();

   // End Project 2 Prototypes
  
   // Start Project 3 Prototypes
   
   void TCPHandler(pack * myMsg);
   void connect(socket_t mySocket);
   socket_t getSocket(uint8_t destP, uint8_t srcP);
   socket_t serverGetSocket(uint8_t destP);
   void connectDone(socket_t mySocket);

   // End Project 3 Prototypes

   event void Boot.booted(){
      call AMControl.start();
      dbg(GENERAL_CHANNEL, "Booted\n");             
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
         initLSTable();
         // findNeighbors();
         call neighborTimer.startPeriodic(1000);      // Random firing timer for neighbor discovery
      }else{
         //Retry until successful
         call AMControl.start();                                            
      }
   }

   event void neighborTimer.fired(){
      findNeighbors();                                // Every time the timer fires, update our list of neighbors and account for dropouts
   }

   event void transmitTimer.fired(){    
       pack p = inFlight;
       tcp_pack * t = (tcp_pack*)(p.payload);
       socket_t mySocket = getSocket(t->srcPort, t->destPort);
       if(mySocket.dest.port){
          dbg(TRANSPORT_CHANNEL, "PACKET DROPPED, RETRANSMITTING PACKET\n");
          call SocketList.pushfront(mySocket);
          makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);

          call transmitTimer.startOneShot(TIMEOUT);

          if(call RoutingTable.get(mySocket.dest.location))
            call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
          else
            dbg(TRANSPORT_CHANNEL, "Can't find route to server...\n"); 
       }    
   }

   event void AMControl.stopDone(error_t err){}

   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
      
      if(len==sizeof(pack)){
         pack* myMsg=(pack*) payload;
         if(checkForPack(myMsg) || myMsg->TTL == 0){ 
             // dbg(GENERAL_CHANNEL, "Dropping the packet\n");
         }else if(myMsg->dest == TOS_NODE_ID){  
  
             //dbg(FLOODING_CHANNEL, "Package Payload: %s :: Package Source: Node %d\n", myMsg->payload, myMsg->src);     // If the packet arrived at the correct destination output the payload   
              
             switch(myMsg->protocol){  

                case PROTOCOL_PING:
                    makePack(&sendPackage, TOS_NODE_ID, myMsg->src, MAX_TTL, PROTOCOL_PINGREPLY, seqNum, 
                            (uint8_t *) myMsg->payload, sizeof(myMsg->payload));
                    seqNum++;

                    addPack(sendPackage);
                    if(call RoutingTable.get(myMsg->src)){
                        dbg(ROUTING_CHANNEL, "Path found, sending reply to next hop %d\n", call RoutingTable.get(myMsg->src));
                        call Sender.send(sendPackage, call RoutingTable.get(myMsg->src));
                    }                  
                    else
                        dbg(ROUTING_CHANNEL, "Path not found, cancelling reply\n");

                    break;                                                                                              // Send a ping reply to the source using flooding                           
            
                case PROTOCOL_PINGREPLY:
                    dbg(FLOODING_CHANNEL, "Received the ping reply from %d\n", myMsg->src);
                    break;                                                                                              // Output if the ping reply was received  

                case PROTOCOL_TCP:
                    TCPHandler(myMsg);
                    break;
            }

         }else if(myMsg->dest == AM_BROADCAST_ADDR){

             switch(myMsg->protocol){
                Neighbor neighbor;                                                                                      // Broadcast packets will help us find neighbors    
                case PROTOCOL_PING:
                    // dbg(NEIGHBOR_CHANNEL, "Received ping from neighbor Node %d\n", myMsg->src);                   
                    makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, MAX_TTL, PROTOCOL_PINGREPLY, 
                            myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));                     

                    addPack(sendPackage);
                    call Sender.send(sendPackage, myMsg->src);
                    break;

                 case PROTOCOL_PINGREPLY:
                    if(!isNeighbor(myMsg->src)){
                        neighbor.srcNode = myMsg->src;
                        neighbor.Age = 0;                                                                             // If the neighbor is not on our list put them on it
                        call NeighborList.pushback(neighbor);
                        LSTable[TOS_NODE_ID - 1][myMsg->src - 1] = 1;
                        sendLSPacket();
                    }
                    break;

                 case PROTOCOL_LINKSTATE:
                    updateLSTable((uint8_t *)myMsg->payload, myMsg->src);                                           // If it's a link state pack update the link state table                                   
                    makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL, myMsg->protocol,
                            myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));

                    addPack(sendPackage);
                    call Sender.send(sendPackage, AM_BROADCAST_ADDR);
                    break;
            }
    
         }else{     
            makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL, myMsg->protocol, myMsg->seq, 
                    (uint8_t *)myMsg->payload, sizeof(myMsg->payload));

            addPack(sendPackage);

            if(call RoutingTable.get(myMsg->dest)){
                // dbg(ROUTING_CHANNEL, "Route found, forwarding to %d\n", call RoutingTable.get(myMsg->dest));
                call Sender.send(sendPackage, call RoutingTable.get(myMsg->dest));
            }else{
                dbg(ROUTING_CHANNEL, "Route not found...\n");
            }
        
         }

             return msg;
     }
         dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
         return msg;
   }   

   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, MAX_TTL, PROTOCOL_PING, seqNum, payload, PACKET_MAX_PAYLOAD_SIZE);
      seqNum++;
      addPack(sendPackage);
    
      if(call RoutingTable.get(destination)){
         dbg(ROUTING_CHANNEL, "Sending to next hop %d\n", call RoutingTable.get(destination));
         call Sender.send(sendPackage, call RoutingTable.get(destination));
      }
      else{
         dbg(ROUTING_CHANNEL, "Route to destination not found...\n");
      }                                                                     
   }

   event void CommandHandler.printNeighbors(){}

   event void CommandHandler.printRouteTable(){
        printRoutingTable();
   }

   event void CommandHandler.printLinkState(){
        printLSTable();
   }

   event void CommandHandler.printDistanceVector(){}


   event void CommandHandler.setTestServer(uint8_t * port){

      socket_t mySocket;
      socket_addr_t myAddr;

      myAddr.location = TOS_NODE_ID;
      myAddr.port = atoi(port);

      mySocket.src = myAddr;

      mySocket.CONN = LISTEN;
        
      mySocket.nextExp = 0; 

      //dbg(TRANSPORT_CHANNEL, "Listening on port: %s\n", port);

      call SocketList.pushfront(mySocket);

   }

   event void CommandHandler.setTestClient(uint8_t * payload){
      
      socket_t mySocket;
      socket_addr_t myAddr;

      uint16_t dest, srcPort, destPort, transfer, i = 0;

      char * token = strtok(payload, ",");
      char buffer[5];
      uint16_t arguments[4];
        
      while(token){
        memcpy(buffer, token, sizeof(token));
        arguments[i] = atoi(token);
        token = strtok(NULL, ",");
        i++;
      }

      dest = arguments[0];
      srcPort = arguments[1];
      destPort = arguments[2];
      transfer = arguments[3];

      myAddr.location = TOS_NODE_ID;
      myAddr.port = srcPort;

      mySocket.dest.port = destPort;
      mySocket.dest.location = dest;
      mySocket.transfer = transfer;

      call SocketList.pushfront(mySocket);
      
      connect(mySocket);

   }

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }

   // Project 1 Functions

   void addPack(pack Package){
      call PacketList.pushback(Package);     // Add the packet to the front of the packet list
   }

   bool checkForPack(pack* Package){
      pack Temp;
      uint16_t i;
      for(i = 0; i < call PacketList.size(); i++){
         Temp = call PacketList.get(i);
         if(Temp.src == Package->src && Temp.seq == Package->seq && Temp.dest == Package->dest){    // If the source/dest and sequence numbers are equal, we've seen this packet before
            return TRUE;
         }

      }

      return FALSE;                                                         // If the packet isn't found, return false
   }

   bool isNeighbor(uint16_t src){
      if(!call NeighborList.isEmpty()){                                     // Check to see if the neighbor exists in the neighbor list
         uint16_t i, size = call NeighborList.size();
         Neighbor neighbor;
         for(i = 0; i < size; i++){
             neighbor = call NeighborList.get(i);
             if(neighbor.srcNode == src){
                 neighbor.Age = 0;
                 return TRUE;
             }
         }

      }

      return FALSE;
   }

   void findNeighbors(){
        char * message;
        Neighbor neighbor, temp;
        uint16_t i, size = call NeighborList.size();
        for(i = 0; i < size; i++){                           // Age each neighbor on each neighbor discovery
            neighbor = call NeighborList.get(i);             // Really messy, but this can't be done with pointers since nesC only allows static allocation
            neighbor.Age++;                                  // Since we have to call a function to retrieve each neighbor, we can't directly change the age value
            call NeighborList.remove(i);                     // i.e. call NeighborList.get(i).Age++;
            call NeighborList.pushback(neighbor);
        }
        for(i = 0; i < size; i++){
            temp = call NeighborList.get(i);
            if(temp.Age > 3){                                // If neighbor is missing for 3 consecutive calls, drop them
                call NeighborList.remove(i);
                size--;
                i--;
            }
        }
        // dbg(NEIGHBOR_CHANNEL, "Sending discovery packets to neighbors...\n");
        message = "foobar";
        makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 2, PROTOCOL_PING, 1,     // Send the neighbor discovery packet(dest = AM_BROADCAST_ADDR) 
                (uint8_t *)message,(uint8_t)sizeof(message));

        addPack(sendPackage);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);
    }

    // End Project 1 Funcitons

    // -----------------------   

    // Project 2 Functions

    void initLSTable(){
        uint16_t i, j;
        for(i = 0; i < MAX; i++){
            for(j = 0; j < MAX; j++){
                    LSTable[i][j] = INFINITY;                           // Initialize all link state table values to infinity(20)
            }
        }
    }

    void sendLSPacket(){
        char payload[255];
        char tempC[127];
        uint16_t i, size = call NeighborList.size();            // Construct the link state packet by concatenating the neighborlist to the payload
        Neighbor neighbor;      
        for(i = 0; i < size; i++){
            neighbor = call NeighborList.get(i);
            sprintf(tempC, "%d", neighbor.srcNode);
            strcat(payload, tempC);
            strcat(payload, ",");
        }
        
        makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 50, PROTOCOL_LINKSTATE, seqNum,
                (uint8_t *) payload, (uint8_t)sizeof(payload));

        seqNum++;
        addPack(sendPackage);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);
    }

    void updateLSTable(uint8_t * payload, uint16_t source){
        uint8_t * temp = payload;
        uint16_t length = strlen((char *)payload);              // Update the link state table neighbor pairs upon receiving a link state packet
        uint16_t i = 0;
        char buffer[5];
        while (i < length){
            if(*(temp + 1) == ','){
                memcpy(buffer, temp, 1);
                temp += 2;
                i += 2;
                buffer[1] = '\0';
            }else if(*(temp + 2) == ','){
               memcpy(buffer, temp, 2);
                temp += 3;
                i += 3;
                buffer[2] = '\0';
            }
            
                LSTable[source - 1][atoi(buffer) - 1] = 1;
        }

        dijkstra();
    }

    void printLSTable(){
        uint16_t i;                                            // Print out the neighbor pairs in the local link state table
        uint16_t j;
        for(i = 0; i < 20; i++){
            for(j = 0; j < 20; j++){
                if(LSTable[i][j] == 1)
                    dbg(ROUTING_CHANNEL, "Neighbors: %d and %d\n", i + 1, j + 1);
            }
        }
    }

    uint16_t minDist(uint16_t dist[], bool sptSet[]){
        uint16_t min = INFINITY, minIndex = 18, i;
        for(i = 0; i < MAX; i++){
            if(sptSet[i] == FALSE && dist[i] < min)
                min = dist[i], minIndex = i;
        }
        return minIndex;
    }

    void dijkstra(){
        uint16_t myID = TOS_NODE_ID - 1, i, count, v, u;
        uint16_t dist[MAX];
        bool sptSet[MAX];
        int parent[MAX];
        int temp;

        for(i = 0; i < MAX; i++){
            dist[i] = INFINITY;
            sptSet[i] = FALSE;
            parent[i] = -1;   
        }

        dist[myID] = 0;

        for(count = 0; count < MAX - 1; count++){
            u = minDist(dist, sptSet);
            sptSet[u] = TRUE;

            for(v = 0; v < MAX; v++){
                if(!sptSet[v] && LSTable[u][v] != INFINITY && dist[u] + LSTable[u][v] < dist[v]){
                    parent[v] = u;
                    dist[v] = dist[u] + LSTable[u][v];
                }
            }           
        }

        for(i = 0; i < MAX; i++){
            temp = i;
            while(parent[temp] != -1  && parent[temp] != myID && temp < MAX){
                temp = parent[temp];
            }
            if(parent[temp] != myID){
                call RoutingTable.insert(i + 1, 0);
            }
            else
                call RoutingTable.insert(i + 1, temp + 1);
        }
    }

    void printRoutingTable(){
        
        uint16_t size = call RoutingTable.size(), i, output;
        for(i = 0; i < size; i++){
            output = call RoutingTable.get((uint32_t) i);
            dbg(ROUTING_CHANNEL, "Key: %d\t Next Hop: %d\n", i, output);
        }

        dbg(ROUTING_CHANNEL, "\n");
    }

    //  End Project 2 Functions

    //-------------------------

    // Start Project 3 Fucntions

    void TCPHandler(pack * myMsg){
        tcp_pack * msg = (tcp_pack *) myMsg->payload;
        uint8_t srcPort, destPort, seq, ACKnum, flag;
        socket_t mySocket;
        uint16_t i, j;
        
        pack p;
        tcp_pack * t;
        srcPort = msg->srcPort;
        destPort = msg->destPort;
        seq = msg->seq;
        ACKnum = msg->ACK;
        flag = msg->flag;
    
        if(flag == SYN_FLAG || flag == SYN_ACK_FLAG || flag == ACK_FLAG){       // Connection setup (three way handshake)

            switch(flag){
           
                case SYN_FLAG:

                    dbg(TRANSPORT_CHANNEL, "RECEIVED SYN!\n"); 
                    mySocket = serverGetSocket(destPort);
                    if(mySocket.src.port && mySocket.CONN == LISTEN){
                        mySocket.CONN = SYN_RCVD;
                        mySocket.dest.port = srcPort;
                        mySocket.dest.location = myMsg->src;
                        call SocketList.pushfront(mySocket);
                        t = (tcp_pack*)(p.payload);
                        t->destPort = mySocket.dest.port;
                        t->srcPort = mySocket.src.port;
                        t->seq = 1;
                        t->ACK = seq + 1;
                        t->flag = SYN_ACK_FLAG;
                        makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
                        if(call RoutingTable.get(mySocket.dest.location))
                            call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
                        else
                            dbg(TRANSPORT_CHANNEL, "Can't find route to client...\n");     
                    }

                    break;

                case SYN_ACK_FLAG:

                    dbg(TRANSPORT_CHANNEL, "RECEIVED SYN_ACK!\n");
                    mySocket = getSocket(destPort, srcPort);      
                    if(mySocket.dest.port){
                        mySocket.CONN = ESTABLISHED;
                        call SocketList.pushfront(mySocket);
                        t = (tcp_pack*)(p.payload);
                        t->destPort = mySocket.dest.port;
                        t->srcPort = mySocket.src.port;
                        t->seq = 1;
                        t->ACK = seq + 1;
                        t->flag = ACK_FLAG;
                        makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
                        if(call RoutingTable.get(mySocket.dest.location))
                            call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
                        else
                            dbg(TRANSPORT_CHANNEL, "Can't find route to server...\n");
        
                        connectDone(mySocket);
    
                    }

                    break;

                case ACK_FLAG:

                    dbg(TRANSPORT_CHANNEL, "ACK RECEIVED, FINALIZING CONNECTION\n");
                    mySocket = getSocket(destPort, srcPort);
                    if(mySocket.src.port && mySocket.CONN == SYN_RCVD){
                        mySocket.CONN = ESTABLISHED;
                        call SocketList.pushfront(mySocket);
                    }

                    break; 
            }
        }

        if(flag == DATA_FLAG || flag == DATA_ACK_FLAG){  // Handle data (ACKS and transmissions)

            if(flag == DATA_FLAG){
            
               dbg(TRANSPORT_CHANNEL, "RECEIVED DATA\n");
               mySocket = getSocket(destPort, srcPort);
               if(mySocket.src.port && mySocket.CONN == ESTABLISHED){             
                   
                   t = (tcp_pack*)(p.payload);
                   if(msg->payload[0] !=  0 && seq == mySocket.nextExp){
                      i = mySocket.lastRCVD + 1;
                      j = 0;
                      while(j < msg->ACK){
                         dbg(TRANSPORT_CHANNEL, "Writing to Receive Buffer: %d\n", i);
                         mySocket.rcvdBuffer[i] = msg->payload[j];
                         mySocket.lastRCVD = msg->payload[j];
                         i++;
                         j++;
                      }
                   }else if(seq == mySocket.nextExp){
                      i = 0;
                      while(i < msg->ACK){
                         dbg(TRANSPORT_CHANNEL, "Writing to Receive Buffer: %d\n", i);
                         mySocket.rcvdBuffer[i] = msg->payload[i];
                         mySocket.lastRCVD = msg->payload[i];
                         i++;
                      }
                   }
    
                   mySocket.advertisedWindow = BUFFER_SIZE - (mySocket.lastRCVD + 1);
                   mySocket.nextExp = seq + 1; 
 
                   call SocketList.pushfront(mySocket);
                   t->destPort = mySocket.dest.port;
                   t->srcPort = mySocket.src.port;
                   t->seq = seq;
                   t->ACK = seq + 1;
                   t->lastACKed = mySocket.lastRCVD;
                   t->advertisedWindow = mySocket.advertisedWindow;
                   t->flag = DATA_ACK_FLAG;
                   makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
         
                   if(call RoutingTable.get(mySocket.dest.location))
                       call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
                   else
                       dbg(TRANSPORT_CHANNEL, "Can't find route to client...\n");     
               }
              
            }

            else if(flag == DATA_ACK_FLAG){
        
              dbg(TRANSPORT_CHANNEL, "RECEIVED DATA ACK, LAST ACKED: %d\n", msg->lastACKed);
              mySocket = getSocket(destPort, srcPort);
              if(mySocket.dest.port && mySocket.CONN == ESTABLISHED){
                if(msg->advertisedWindow != 0 && msg->lastACKed != mySocket.transfer){
                    dbg(TRANSPORT_CHANNEL, "SENDING NEXT DATA\n");
                    
                    t = (tcp_pack*)(p.payload);
                    i = msg->lastACKed + 1;
                    j = 0;
                    while(j < msg->advertisedWindow && j < TCP_MAX_PAYLOAD_SIZE && i <= mySocket.transfer){
                        dbg(TRANSPORT_CHANNEL, "Writing to Payload: %d\n", i);
                        t->payload[j] = i;
                        i++; 
                        j++;
                    } 
                     
                    call SocketList.pushfront(mySocket);        
                    t->flag = DATA_FLAG;
                    t->destPort = mySocket.dest.port;
                    t->srcPort = mySocket.src.port;
                    t->ACK = (i - 1) - msg->lastACKed;;
                    t->seq = ACKnum;
             
                    makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);     
                                        
                    makePack(&inFlight, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
                    
                    call transmitTimer.startOneShot(TIMEOUT);
            
                    if(call RoutingTable.get(mySocket.dest.location)){
                        call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
                    }
                    else{
                        dbg(ROUTING_CHANNEL, "Route to destination server not found...\n");
                    }
  
                }else{

                    dbg(TRANSPORT_CHANNEL, "ALL DATA SENT, CLOSING CONNECTION\n");                  
                    mySocket.CONN = FIN_WAIT1;
                    call SocketList.pushfront(mySocket);
                    t = (tcp_pack*)(p.payload);
                    t->destPort = mySocket.dest.port;
                    t->srcPort = mySocket.src.port;
                    t->seq = 1;
                    t->ACK = seq + 1;
                    t->flag = FIN_FLAG;
                    makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
                    if(call RoutingTable.get(mySocket.dest.location))
                        call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
                    else
                        dbg(TRANSPORT_CHANNEL, "Can't find route to server...\n");
               }
            }
          }
        }

        if(flag == FIN_FLAG || flag == ACK_FIN_FLAG){   // Handle connection teardown

            if(flag == FIN_FLAG){

                dbg(TRANSPORT_CHANNEL, "RECEIVED FIN REQUEST\n");
                mySocket = getSocket(destPort, srcPort);
                if(mySocket.src.port){
                    mySocket.CONN = CLOSED;
                    mySocket.dest.port = srcPort;
                    mySocket.dest.location = myMsg->src;
                    //call SocketList.pushfront(mySocket); Don't add the socket to the list again so it's basically dropped
                    t = (tcp_pack*)(p.payload);
                    t->destPort = mySocket.dest.port;
                    t->srcPort = mySocket.src.port;
                    t->seq = 1;
                    t->ACK = seq + 1;
                    t->flag = ACK_FIN_FLAG;

                    dbg(TRANSPORT_CHANNEL, "CONNECTION CLOSING, DATA RECEIVED: \n");

                    for(i = 0; i <= mySocket.lastRCVD; i++){
                        dbg(TRANSPORT_CHANNEL, "%d\n", mySocket.rcvdBuffer[i]);
                    }
                 
                    makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
                    if(call RoutingTable.get(mySocket.dest.location))
                        call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
                    else
                        dbg(TRANSPORT_CHANNEL, "Can't find route to client...\n");     
                 }
            }

            if(flag == ACK_FIN_FLAG){
            
                dbg(TRANSPORT_CHANNEL, "RECEIVED FIN ACK, GOODBYE\n");
                mySocket = getSocket(destPort, srcPort);      
                if(mySocket.dest.port){
                    mySocket.CONN = CLOSED;
                    //call SocketList.pushfront(mySocket); Don't add the socket to the list again so it's basically dropped
                }
            }
        }
    }

    void connect(socket_t mySocket){          // Initial conneciton client->server
        pack p; 
        tcp_pack * TCPpack;
        TCPpack = (tcp_pack*)(p.payload);
        TCPpack->flag = SYN_FLAG;
        TCPpack->destPort = mySocket.dest.port;
        TCPpack->srcPort = mySocket.src.port;
        TCPpack->ACK = 0;
        TCPpack->seq = 1;

        makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, TCPpack, PACKET_MAX_PAYLOAD_SIZE);

        mySocket.CONN = SYN_SENT;
           
        if(call RoutingTable.get(mySocket.dest.location)){
           call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
        }
        else{
           dbg(ROUTING_CHANNEL, "Route to destination server not found...\n");
        }

    }

    void connectDone(socket_t mySocket){        // Done connecting, start sending data
        uint16_t i;
        pack p;
        tcp_pack * t;
        t = (tcp_pack *)(p.payload);
        t->flag = DATA_FLAG;
        t->destPort = mySocket.dest.port;
        t->srcPort = mySocket.src.port;
        t->seq = 0;

        i = 0;
        while(i < TCP_MAX_PAYLOAD_SIZE && i <= mySocket.transfer){        // This makes the assumption that we can send a full TCP payload initially(6 16 bit ints)
            t->payload[i] = i;
            i++;
        }
        
        t->ACK = i;

        makePack(&p, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
                      
        makePack(&inFlight, TOS_NODE_ID, mySocket.dest.location, MAX_TTL, PROTOCOL_TCP, 0, t, PACKET_MAX_PAYLOAD_SIZE);
 
        call transmitTimer.startOneShot(TIMEOUT);
                 
        if(call RoutingTable.get(mySocket.dest.location)){
            call Sender.send(p, call RoutingTable.get(mySocket.dest.location));
        }
        else{
            dbg(ROUTING_CHANNEL, "Route to destination server not found...\n");
        }
    }

    socket_t getSocket(uint8_t destP, uint8_t srcP){         // Find the socket in the list w/ proper srcPort and destPort
        bool sockFound;
        socket_t temp;
        uint16_t i, size = call SocketList.size();
        for(i = 0; i < size; i++){
            temp = call SocketList.get(i);
            if(temp.src.port == destP && temp.dest.port == srcP && temp.CONN != LISTEN){
                sockFound = TRUE;
                call SocketList.remove(i);
                break;
            }

        }
        
        if(sockFound)
            return temp;
        else
            dbg(TRANSPORT_CHANNEL, "Socket not found...\n");
    }

    socket_t serverGetSocket(uint8_t destP){   // Server finds the listening socket, then opens up a new socket to host the new connection
        bool sockFound;
        socket_t temp;
        uint16_t i, size = call SocketList.size();
        for(i = 0; i < size; i++){
            temp = call SocketList.get(i);
            if(temp.src.port == destP && temp.CONN == LISTEN){
                sockFound = TRUE;
                break;
            }

        }
        
        if(sockFound)
            return temp;
        else
            dbg(TRANSPORT_CHANNEL, "Socket not found...\n");

    }

    // End Project 3 Functions
 
}
