from TestSim import TestSim

def main():
    # Get simulation ready to run.
    s = TestSim();

    # Before we do anything, lets simulate the network off.
    s.runTime(1);

    # Load the the layout of the network.
    s.loadTopo("test.topo");

    # Add a noise model to all of the motes.
    s.loadNoise("no_noise.txt");

    # Turn on all of the sensors.
    s.bootAll();

    # Add the main channels. These channels are declared in includes/channels.h
    s.addChannel(s.COMMAND_CHANNEL);
    s.addChannel(s.GENERAL_CHANNEL);
    s.addChannel(s.FLOODING_CHANNEL);
    s.addChannel(s.NEIGHBOR_CHANNEL);
    s.addChannel(s.ROUTING_CHANNEL);
    s.addChannel(s.TRANSPORT_CHANNEL);

    # After sending a ping, simulate a little to prevent collision.
    s.runTime(10);
    
    s.testServer(2, "4");
    s.runTime(5);

    s.testClient(4, "2,1,4,78");
    s.runTime(50);

if __name__ == '__main__':
    main()
