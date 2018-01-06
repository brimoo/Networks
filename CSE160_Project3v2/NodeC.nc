/**
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */

#include <Timer.h>
#include "includes/CommandMsg.h"
#include "includes/packet.h"
#include "includes/socket.h"
#include "includes/TCPpacket.h"

configuration NodeC{
}
implementation {
    components MainC;
    components Node;
    components RandomC as Random;
    components new TimerMilliC() as neighborTimer;
    components new TimerMilliC() as transmitTimer;
    components new AMReceiverC(AM_PACK) as GeneralReceive;

    Node.neighborTimer -> neighborTimer;

    Node.transmitTimer -> transmitTimer;
 
    Node -> MainC.Boot;

    Node.Receive -> GeneralReceive;

    Node.Random -> Random;

    components ActiveMessageC;
    Node.AMControl -> ActiveMessageC;

    components new SimpleSendC(AM_PACK);
    Node.Sender -> SimpleSendC;

    components new HashmapC(uint16_t, 64) as RoutingTableC;
    Node.RoutingTable -> RoutingTableC; 

    components new ListC(pack, 64) as PacketListC;
    Node.PacketList -> PacketListC;

    components new ListC(Neighbor, 64) as NeighborsC;
    Node.NeighborList -> NeighborsC;

    components new ListC(socket_t, 10) as SocketList;
    Node.SocketList -> SocketList;

    components CommandHandlerC;
    Node.CommandHandler -> CommandHandlerC;
}
