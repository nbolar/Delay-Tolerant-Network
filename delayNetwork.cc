/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/aodv-module.h"
using namespace ns3;

uint32_t bytesTotal;
uint32_t packetsReceived;
uint32_t Node1SendAck = 0;
uint32_t Node2SendAck = 0;

uint16_t Node1Data[50] ;
uint16_t Node2Data = 0;
uint16_t receivedpacketN2_head[50];
uint16_t globalcounter = 0;
uint16_t ackNode0 = 0;
uint16_t counter = 0;
uint16_t new_counter = 0;
uint16_t pos = 0;
uint16_t pos2 = 0;
uint16_t pos3 = 0;
uint16_t check = 0;

uint32_t packetsID = 0;

NS_LOG_COMPONENT_DEFINE ("FinalProject2017-18");

uint16_t node1_globalcounter[50] ;
uint16_t node1_isbufferempty = 1;
uint16_t node2_globalcounter = 0;
uint16_t node2_isbufferempty = 1;

//----Experiment parameters

double distance_between_node0_node2 = 2000; //x-axis distance
double distance_between_node0_node1 = 10; //y-axis distance


//--------------------Custom header code begin------------//

class MyHeader : public Header
{
public:
	void SetData (uint16_t data);
	uint16_t GetData (void);
	void SetPacketType (uint16_t data);
	uint16_t GetPacketType (void);
	void Setisbufferempty (uint16_t data);
	uint16_t Getisbufferempty (void);
	void Setglobalcounter (uint16_t data);
	uint16_t Getglobalcounter (void);
	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (Buffer::Iterator start) const;
	virtual uint32_t Deserialize (Buffer::Iterator start);
	virtual void Print (std::ostream &os) const;
private:
	uint16_t m_data;
	uint16_t m_packettype;
	uint16_t m_isbufferempty;
	uint16_t m_globalcounter;
};

//Implementation of public members of class MyHeader
void
MyHeader::SetData (uint16_t data){
	m_data = data;
}

uint16_t
MyHeader::GetData (void){
	return m_data;
}

void
MyHeader::SetPacketType (uint16_t data){
	m_packettype = data;
}

uint16_t
MyHeader::GetPacketType (void){
	return m_packettype;
}

void
MyHeader::Setglobalcounter (uint16_t data){
	m_globalcounter = data;
}

uint16_t
MyHeader::Getglobalcounter (void){
	return m_globalcounter;
}

uint32_t
MyHeader::GetSerializedSize (void) const{
		return 12;
}

void
MyHeader::Serialize (Buffer::Iterator start) const{
	start.WriteHtonU16 (m_data);
	start.WriteHtonU16 (m_packettype);
	start.WriteHtonU16 (m_isbufferempty);
	start.WriteHtonU16 (m_globalcounter);
}

uint32_t
MyHeader::Deserialize (Buffer::Iterator start){
	m_data = start.ReadNtohU16 ();
	m_packettype = start.ReadNtohU16 ();
	m_isbufferempty = start.ReadNtohU16 ();
	m_globalcounter = start.ReadNtohU16 ();
	return 12;
}

void
MyHeader::Print (std::ostream &os) const{
	os << m_data;
}

TypeId
MyHeader::GetTypeId (void){
	static TypeId tid = TypeId ("ns3::MyHeader")
		.SetParent<Header> ()
		.AddConstructor<MyHeader> ()
		;
	return tid;
}

TypeId
MyHeader::GetInstanceTypeId (void) const{
	return GetTypeId ();
}

//--------------------Custom header code end------------//


//This function keeps track of the position of a moving node using (x,y) coordinates
void CourseChangeSink (Ptr<OutputStreamWrapper> stream, std::string context, Ptr<const MobilityModel> model){
	Vector position = model->GetPosition ();
	NS_LOG_UNCOND (Simulator::Now ().GetSeconds () <<	" x = " << position.x << ", y = " << position.y);
	*stream->GetStream () << Simulator::Now ().GetSeconds () << " x = " << position.x << ", y = " << position.y << std::endl;
}

//---High level code for Node 0 BEGIN

static void Node0DataGen (){
	if (ackNode0 == 1){
		ackNode0 = 0;
		globalcounter ++;
	}
	Simulator::Schedule (Seconds(1.00), &Node0DataGen);
}

static void Node0SendPacket (Ptr<Socket> socket, uint32_t pktSize){
	Ptr<Packet> p = Create<Packet> ();
	MyHeader XXheader;
	if (ackNode0 == 0 && globalcounter < 50){
    XXheader.SetPacketType (1); // 1 for data
    XXheader.SetData (globalcounter);	// header fields
    XXheader.Setglobalcounter (globalcounter);	// header fields
    p->AddHeader (XXheader);
    socket->Send (p);
	}
	Simulator::Schedule (Seconds(0.25), &Node0SendPacket, socket, pktSize); //Packet is sent constantly until acknowledged
}


static void Node0ReceivePacket (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket){
	NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t<< Node 0 ACK Received.");
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << "Node 0 ACK Received" << std::endl;
	ackNode0 = 1;
	// Node1SendAck = 1;
}
//---High level code for node 0 END

//---High level code for node 1 BEGIN
static void Node1ReceivePacket (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket){

	MyHeader header;
	uint16_t type_of_packet;
	Ptr<Packet> packet;
	packet = socket->Recv();
	packet->RemoveHeader(header);
	type_of_packet = header.GetPacketType();

	if (type_of_packet == 1 && new_counter ==0){ //data
    Node1SendAck = 1;
		new_counter += 1;
    node1_globalcounter[pos] = header.Getglobalcounter ();	// header fields
    Node1Data[pos] = header.GetData ();	// header fields
	  NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t<< Node 1 received a packet " << "[D]::[" << Node1Data[pos]<< "]");
		*stream->GetStream () << Simulator::Now ().GetSeconds () <<  "\t<< Node 1 received a packet " << "[D]::[" << Node1Data[pos] << "]"<<std::endl;
		pos += 1;
	}
	else if (type_of_packet == 2){
	Node1SendAck = 0;
	NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t>> Node 1 received an ACK.");
	// *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << "Node 1 ACK Received" << std::endl;
	// pos2 += 1;
		}
	else if(type_of_packet == 1){
		// node1_globalcounter[pos] = header.Getglobalcounter ();	// header fields
    // Node1Data[pos] = header.GetData ();	// header fields
			if ((header.GetData () != Node1Data[pos-1]) && (header.Getglobalcounter() != node1_globalcounter[pos-1]) && (pos<50)){
			Node1SendAck = 1;
			node1_globalcounter[pos] = header.Getglobalcounter ();	// header fields
			Node1Data[pos] = header.GetData ();	// header fields
			NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t<< Node 1 received a packet " << "[D]::[" << Node1Data[pos]<< "]");
			*stream->GetStream () << Simulator::Now ().GetSeconds () <<  "\t<< Node 1 received a packet " << "[D]::[" << Node1Data[pos] << "]"<<std::endl;
			pos += 1;
		}

	}

}

static void Node1SendPacket (Ptr<Socket> socket, uint32_t pktSize){
	Ptr<Packet> p = Create<Packet> ();
	MyHeader XXheader;
	// std::cout << Node1SendAck << '\n'
  if(Node1SendAck == 0){
		//NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t>> Node 1 sending packet.");
    XXheader.SetPacketType (1); // 1 for data
    XXheader.SetData (Node1Data[pos2]);	// header fields
    XXheader.Setglobalcounter (node1_globalcounter[pos2]);	// header fields
    p->AddHeader (XXheader);
    socket->Send (p);

  }
  Simulator::Schedule (Seconds(0.25), &Node1SendPacket, socket, pktSize); //Packet is sent constantly until acknowledged
}

static void Node1AckLoop (Ptr<Socket> socket, Ptr<Socket> socket2, uint32_t pktSize){
	// std::cout << Node1SendAck << '\n';
	if (Node1SendAck == 1){
		NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t>> Node 1 sending an ACK.");
		Node1SendAck = 0;
		socket->Send (Create<Packet> (pktSize));
		Simulator::Schedule (Seconds (0.0001), &Node1SendPacket, socket2, pktSize);
	}
	Simulator::Schedule (Seconds(0.01), &Node1AckLoop, socket, socket2, pktSize);

}
//---High level code for node 1 END

//---High level code for node 2 BEGIN

static void Node2ReceivePacket (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket){
	MyHeader header;
	Ptr<Packet> packet = socket->Recv();
	packet->RemoveHeader(header);
	receivedpacketN2_head[pos3] = header.GetData();
	uint16_t type_of_packet = header.GetPacketType();

  if(type_of_packet == 1 && counter == 0){
	 Node2SendAck = 1;
	 counter += 1;
	//  std::cout << "if" << '\n';
		NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t<< Node 2 received a packet " << "[D]::[" << receivedpacketN2_head[pos3] << "]");
		*stream->GetStream () << Simulator::Now ().GetSeconds () <<  "\t<< Node 2 received a packet " << "[D]::[" << receivedpacketN2_head[pos3] << "]"<<std::endl;
		pos3 += 1;
		pos2 +=1;
	}
	else if((type_of_packet = 1) && (receivedpacketN2_head[pos3] != receivedpacketN2_head[pos3-1]) && (receivedpacketN2_head[pos3]!=0))
	{
		Node2SendAck = 1;
		NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t<< Node 2 received a packet " << "[D]::[" << receivedpacketN2_head[pos3] << "]");
		*stream->GetStream () << Simulator::Now ().GetSeconds () <<  "\t<< Node 2 received a packet " << "[D]::[" << receivedpacketN2_head[pos3] << "]"<<std::endl;
		pos3 +=1;
		pos2 +=1;
	}


}

static void Node2AckLoop (Ptr<Socket> socket, uint32_t pktSize){
	Ptr<Packet> p = Create<Packet> ();
	MyHeader XXheader;

	if (Node2SendAck == 1){
		NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "\t>> Node 2 sending an ACK.");
    		Node2SendAck = 0;
		//Node1SendAck =1;
		XXheader.SetPacketType (2);
		p->AddHeader (XXheader);
		socket->Send (p);
	}
	Simulator::Schedule (Seconds(0.01), &Node2AckLoop, socket, pktSize);
}

//---High level code for node 2 END

//Main Function
int main (int argc, char *argv[]){

	double txp = 1.5; //transmission power dB
	std::string phyMode ("DsssRate1Mbps");
  double duration = 200 ; // Simulation length

	//Set Non-unicastMode rate to unicast mode
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",  UintegerValue(50));
  Config::SetDefault("ns3::WifiRemoteStationManager::MaxSlrc",  UintegerValue(1000));
  Config::SetDefault("ns3::WifiRemoteStationManager::MaxSsrc",  UintegerValue(1000));


	// 1. Create the nodes
	NodeContainer adhocNodes;
	adhocNodes.Create(3);	//All 3 nodes are members of adhocNodes
	NodeContainer mobileNode = NodeContainer (adhocNodes.Get(1)); //Node 1 is defined as the mobileNode (i.e., the bus)
	NodeContainer stationaryNodes = NodeContainer (adhocNodes.Get(0), adhocNodes.Get(2)); //Node 0 and Node 2 are defined as stationaryNodes (i.e., the villages)

	// 2. Set up physical layer
	WifiHelper wifi;
	wifi.SetStandard (WIFI_PHY_STANDARD_80211b); //Use 802.11 standard
	YansWifiPhyHelper wifiPhy =	YansWifiPhyHelper::Default ();
	wifiPhy.Set ("RxGain", DoubleValue (-10) ); //Set up the gain at the receiver (in dB)

	// 3. Set up propagation loss model
	YansWifiChannelHelper wifiChannel;

  std::string lossModel = "ns3::LogDistancePropagationLossModel"; //Set up the propagation loss model
  std::string atr1 = "Exponent"; //Set up exponent value of the Path Loss propagation model
  std::string atr2 = "ReferenceDistance";// The distance at which the reference loss is calculated (m)
  std::string atr3 = "ReferenceLoss";// The reference loss at reference distance (dB)

  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel"); //Set up the propagation delay model
  wifiChannel.AddPropagationLoss (lossModel, atr1, DoubleValue(2.5), atr2, DoubleValue(1), atr3, DoubleValue(11));

	wifiPhy.SetChannel (wifiChannel.Create ());

	wifiPhy.Set ("TxPowerStart",DoubleValue (txp)); //set up the transmission power
	wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));


	// 4. Set up the MAC layer and install the wireless devices
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
	wifi.SetRemoteStationManager(
    "ns3::ConstantRateWifiManager",
		"DataMode",StringValue ("DsssRate1Mbps"),
		"ControlMode",StringValue ("DsssRate1Mbps"),
    "MaxSlrc",StringValue ("1000000"));
	wifiMac.SetType ("ns3::AdhocWifiMac"); //Set MAC to ad hoc mode
	NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

	// 5. Set up mobility model for Node 1
	MobilityHelper mobilityMobileNode;

	Ptr<ListPositionAllocator> positionAllocMobileNode = CreateObject<ListPositionAllocator> ();
	positionAllocMobileNode->Add (Vector (0.0, distance_between_node0_node1, 0.0)); //3D coordinates (x,y,z). Use only the x-y plane
	mobilityMobileNode.SetPositionAllocator (positionAllocMobileNode);

	mobilityMobileNode.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
		"TimeStep", TimeValue (Seconds (1)),
		"Alpha",DoubleValue(1),
		"MeanVelocity", StringValue ("ns3::ConstantRandomVariable[Constant=25]"), //This is where we set the speed of node 1 (m/s)
		"MeanDirection", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
		"MeanPitch", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
		"NormalVelocity", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
		"NormalDirection", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
		"NormalPitch", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
		"Bounds",BoxValue (Box (0, distance_between_node0_node2, distance_between_node0_node1, distance_between_node0_node1, 0, 0))); //This is the size of the simulated area.
  mobilityMobileNode.Install (mobileNode);

	// 6. Connect trace source to trace sink for Node 1
	std::ostringstream oss;
	oss << "/NodeList/" << mobileNode.Get (0)->GetId () << "/$ns3::MobilityModel/CourseChange";
	AsciiTraceHelper asciiTraceHelper;
	Ptr<OutputStreamWrapper> locationStream = asciiTraceHelper.CreateFileStream (".~");
	Config::Connect (oss.str (), MakeBoundCallback (&CourseChangeSink, locationStream));

	// 7. Assign positions to Node 0 and Node 2
	MobilityHelper mobilityStaNodes;
	mobilityStaNodes.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	Ptr<ListPositionAllocator> positionAllocStaNodes = CreateObject<ListPositionAllocator> ();
	positionAllocStaNodes->Add (Vector (0.0, 0.0, 0.0)); //3D coordinates (x,y,z). Use only the x-y plane
	positionAllocStaNodes->Add (Vector (distance_between_node0_node2, 0.0, 0.0)); //3D coordinates (x,y,z). Use only the x-y plane
	mobilityStaNodes.SetPositionAllocator(positionAllocStaNodes);
	mobilityStaNodes.Install (stationaryNodes);

	// 8. Set up	the routing protocol (AODV)
	AodvHelper aodv;
	Ipv4ListRoutingHelper list;
	InternetStackHelper internet;
	aodv.Set("EnableHello", BooleanValue(false));
	aodv.Set("GratuitousReply", BooleanValue(false));
	aodv.Set("ActiveRouteTimeout", ns3::TimeValue(Seconds(600)));
	list.Add (aodv, 100);

	internet.SetRoutingHelper (list);
	internet.Install (adhocNodes);

	// 9. Assign IP addresses to nodes
	Ipv4AddressHelper address;
	address.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer interfaces;
	interfaces = address.Assign (devices);

	// 10. Reception of data; we connect the receiving sockets of each node to their respective callbacks
	// These callbacks basically define what a node does next upon receiving a packet

	//---node 0  BEGIN
	TypeId tid0 = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> Node0Recv = Socket::CreateSocket (adhocNodes.Get (0), tid0);
	InetSocketAddress local0 = InetSocketAddress (Ipv4Address::GetAny (), 9);
	Node0Recv->Bind (local0);
	Ptr<OutputStreamWrapper> Node0rcvdStream = asciiTraceHelper.CreateFileStream ("Node0_Rcvd.md");
	Node0Recv->SetRecvCallback (MakeBoundCallback (&Node0ReceivePacket, Node0rcvdStream));
	//---node 0 END

	//---node 1 BEGIN
	TypeId tid1 = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> Node1Recv = Socket::CreateSocket (adhocNodes.Get (1), tid1);
	InetSocketAddress local1 = InetSocketAddress (Ipv4Address::GetAny (), 9);
	Node1Recv->Bind (local1);
	Ptr<OutputStreamWrapper> Node1rcvdStream = asciiTraceHelper.CreateFileStream ("Node1_Rcvd.md");
	Node1Recv->SetRecvCallback (MakeBoundCallback (&Node1ReceivePacket, Node1rcvdStream));
	//---node 1 END

	//---node 2  BEGIN
	TypeId tid2 = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> Node2Recv = Socket::CreateSocket (adhocNodes.Get (2), tid2);
	InetSocketAddress local2 = InetSocketAddress (Ipv4Address::GetAny (), 9);
	Node2Recv->Bind (local2);
	Ptr<OutputStreamWrapper> Node2rcvdStream = asciiTraceHelper.CreateFileStream ("Node2_Rcvd.md");
	Node2Recv->SetRecvCallback (MakeBoundCallback (&Node2ReceivePacket, Node2rcvdStream));
	//---node 2 END

	// 11. Generate traffic
	uint32_t packetSize = 200; //bytes

	//Start the node 0 ---> node 1 connection
	Ptr<Socket> source = Socket::CreateSocket (adhocNodes.Get (0), tid0);
	InetSocketAddress SocketAddressof1 = InetSocketAddress (interfaces.GetAddress (1, 0), 9);
	source->Connect (SocketAddressof1);
	Simulator::Schedule (Seconds (0.0001), &Node0SendPacket, source, packetSize);

	//Start node 1's ACK loop (node 1 ---> node 0, socket 9)
	//Define the node 1---> node 2 connection here as well (socket 9)
	//We need 2 sockets: the first one is used by Node 1 to send ACKs to Node 0
	//The second one is used by Node 1 the send the packet to Node 2
	Ptr<Socket> sourceSocketforNode1 = Socket::CreateSocket (adhocNodes.Get (1), tid1);
	InetSocketAddress SocketAddressof0 = InetSocketAddress (interfaces.GetAddress (0, 0), 9);
	sourceSocketforNode1->Connect (SocketAddressof0);

	Ptr<Socket> sourceSocketforNode1SECOND = Socket::CreateSocket (adhocNodes.Get (1), tid1);
	InetSocketAddress SocketAddressof2 = InetSocketAddress (interfaces.GetAddress (2, 0), 9);
	sourceSocketforNode1SECOND->Connect (SocketAddressof2);

	Simulator::Schedule (Seconds (0.0001), &Node1AckLoop, sourceSocketforNode1, sourceSocketforNode1SECOND, packetSize);

	//Start node 2's ACK loop (node 2---> node 1, socket 9)
	Ptr<Socket> sourceSocketforNode2 = Socket::CreateSocket (adhocNodes.Get (2), tid2);
	sourceSocketforNode2->Connect (SocketAddressof1);
	Simulator::Schedule (Seconds (0.0001), &Node2AckLoop, sourceSocketforNode2, packetSize);



	Simulator::Schedule (Seconds(0.0001), &Node0DataGen);


	// 12. Run the simulation
	Simulator::Stop (Seconds (duration));
	Simulator::Run();
	Simulator::Destroy ();
	// std::cout << receivedpacketN2_head[] << '\n';
	return 0;
}
