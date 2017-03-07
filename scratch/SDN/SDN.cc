/*
 * SDN.cc
 *
 *  Created on: Oct 9, 2015
 *      Author: chl
 */
/*
  ./waf --run "SDN"
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <dirent.h>//DIR*
#include "SDN.h"
#include <string> //config connect

NS_LOG_COMPONENT_DEFINE ("SDN");


using namespace ns3;

VanetSim::VanetSim()
{
	logFile = "SDN.log";
	//wifi network setting
	phyMode = "OfdmRate27MbpsBW10MHz";//wifi phymode
	lossModle = "ns3::FriisPropagationLossModel";//distance could not be 0?
	delayModle = "ns3::ConstantSpeedPropagationDelayModel";//use ConstantSpeedPropagationDelayModel means the affect of distance is much smaller than the delay in MAC or other place, because speed is set to C(Speed of light)
	//delayModle = "ns3::RandomPropagationDelayModel";

	freq1 = 5.860e9;  //802.11p SCH CH172
	freq2 = 5.890e9;  //802.11p CCH CH178
	txp1 = 23;  // dBm SCH power
	txp2 = 31;  // CCH power
	range1 = 400.0;//SCH
	range2 = 1100.0;//CCH
	packetSize = 1000; // bytes for SCH
	//numPackets = 1;
	//interval = 0.1; // seconds
	verbose = false;//turn on all WifiNetDevice log components


	//input
	//traceFile = "";// use for ns 2
	homepath = ".";//getenv("HOME");
	folder="SDNData";
	duration = -1;
	mod = 1;
	nodeNum = 0;
	numofLC = 4;
	numofSource = 3;
	numofSink = 3;

	//source communication port
	//statistic
	base_port=65419;
	for(int i=0;i<numofSource;i++)
	{
		m_port.push_back(base_port+i);
		transmissionStatistics  temp;
		temp.Rx_Data_Bytes=0;
		temp.Rx_Data_Pkts=0;
		temp.Tx_Data_Bytes=0;
		temp.Tx_Data_Pkts=0;
		temp.Unique_RX_Pkts=0;
		temp.old_Rx_Data_Pkts=0;
		temp.old_Tx_Data_Pkts=0;
		temp.old_Unique_RX_Pkts=0;

		statistics.push_back(temp);
	}
	m_numofhm = 0;
	m_numofapp = 0;
	m_numofackhello = 0;
	m_numofdontforward = 0;
	m_numoflc2lc = 0;
	m_numofdatapacket = 0;
	m_numofallforwardcar = 0;
}

VanetSim::~VanetSim()
{
	os.close();
	dos.close();
}

void VanetSim::Simulate(int argc, char *argv[])
{
     std::ios_base::sync_with_stdio (false);
	SetDefault();
	ParseArguments(argc, argv);
	LoadTraffic();
	ConfigNode();
	ConfigChannels();
	ConfigDevices();
	ConfigMobility();
	ConfigApp();
	ConfigTracing();
	Run();
	ProcessOutputs();
	 std::cout<<std::endl;
}

void VanetSim::SetDefault()
{
	//Handle By Constructor partly

	// disable fragmentation for frames below 2200 bytes. also useless because all the packet is set to constant size: SCH 1000 CCH:very small.
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
	// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

	// turn off RTS/CTS for frames below 2200 bytes
	//soluting the hidden node problem. by default, the CSMA/CA is swtich on in the wifi protocal. So this options is useless.
	UintegerValue ctsThr = 0;
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
}

void VanetSim::ParseArguments(int argc, char *argv[])
{
	CommandLine cmd;
//	cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
//	cmd.AddValue ("nodeNum", "Number of nodes", nodeNum);
	cmd.AddValue ("duration", "Duration of Simulation", duration);
//	cmd.AddValue ("logFile", "Log file", logFile);
	cmd.AddValue ("folder", "Working Directory", folder);
	cmd.AddValue ("txp1", "TX power for SCH", txp1);
	cmd.AddValue ("txp2", "TX power for CCH", txp2);
	cmd.AddValue ("range1", "Range for SCH", range1);
	cmd.AddValue ("range2", "Range for CCH", range2);
	cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
	cmd.AddValue ("mod", "0=olsr 1=bs-sdn(DEFAULT) 2=aodv 3=dsdv 4=dsr 5=yangs-sdn", mod);
	cmd.AddValue ("ds", "DataSet", m_ds);
	cmd.Parse (argc,argv);

	switch (mod)
	{
	  case 2:
      m_todo = "AODV";
      break;
    case 3:
      m_todo = "DSDV";
      break;
    case 4:
      m_todo = "DSR";
      break;
    case 0:
      m_todo = "OLSR";
      break;
    case 5:
      m_todo = "YANGS-SDN";
      break;
    default:
      m_todo = "BS-SDN";
      mod = 1;
      break;
	}
}

void VanetSim::LoadTraffic()
{
	DIR* dir = NULL;
	//DIR* subdir=NULL;
	std::string temp(homepath+"/"+folder);
	if((dir = opendir(temp.data()))==NULL)
		NS_FATAL_ERROR("Cannot open input path "<<temp.data()<<", Aborted.");

	//read the input map
	std::string sumo_net = temp + "/input.net.xml";
	std::string sumo_route = temp + "/input.rou.xml";
	std::string sumo_fcd = temp + "/input.fcd.xml";
	ns3::vanetmobility::VANETmobilityHelper mobilityHelper;
	VMo=mobilityHelper.GetSumoMObility(sumo_net,sumo_route,sumo_fcd);
	nodeNum = VMo->GetNodeSize();

	std::string output = temp + "/" + m_todo + "_" + m_ds + "_result_new.txt";
	std::string delay_output = temp + "/" + m_todo + "_" + m_ds +"_delay.txt";
	os.open(output.data(),std::ios::out);
	dos.open(delay_output.data(),std::ios::out);

	//cout log
     std::cout<<"Mode: "<<m_todo<<"DataSet:  "<<m_ds<<" "<<nodeNum<<std::endl;
     os<<"Mode:  "<<m_todo<<"DataSet:  "<<m_ds<<std::endl;
     dos<<"Mode: "<<m_todo<<"  DataSet:  "<<m_ds<<std::endl;
}



void VanetSim::ConfigNode()
{
	m_nodes.Create(nodeNum+32);//Cars + LC1-8+Source1-12+Sink1-12
	/*Only Apps Are Different Between Different kind of Nodes*/
	// Name nodes
	for (uint32_t i = 0; i < nodeNum; ++i)
	{
		 std::ostringstream os;
		 os << "vehicle-" << i;
		Names::Add(os.str(), m_nodes.Get(i));
	}
	for (uint32_t i = 1; i <= numofLC; ++i)
	{
		std::ostringstream os;
		os << "LC" << i;
		Names::Add(os.str(), m_nodes.Get(nodeNum - 1 + i));
	}
	for (uint32_t i = 1; i <= numofSource; ++i)
	{
		std::ostringstream os;
		os << "Source" << i;
		Names::Add(os.str(), m_nodes.Get(nodeNum - 1 + numofLC + i));
	}
	for (uint32_t i = 1; i <= numofSink; ++i)
	{
		std::ostringstream os;
		os << "Sink" << i;
		Names::Add(os.str(),
				m_nodes.Get(nodeNum - 1 + numofLC + numofSource + i));
	}
}

void VanetSim::ConfigChannels()
{
	//===channel
	YansWifiChannelHelper SCHChannel;
	SCHChannel.SetPropagationDelay (delayModle);
	SCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq1));
	//SCHChannel.AddPropagationLoss("ns3::FriisPropagationLossModel","Frequency", DoubleValue(freq1));
	YansWifiChannelHelper CCHChannel;
	CCHChannel.SetPropagationDelay (delayModle);
	CCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq2));


	// the channelg
	Ptr<YansWifiChannel> SCH = SCHChannel.Create();
	Ptr<YansWifiChannel> CCH = CCHChannel.Create();

	//===wifiphy
	m_SCHPhy =  YansWifiPhyHelper::Default ();
	m_SCHPhy.SetChannel (SCH);
	m_SCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);//pcap data link types
	m_CCHPhy =  YansWifiPhyHelper::Default ();
	m_CCHPhy.SetChannel (CCH);
	m_CCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

	// 802.11p mac
	NqosWaveMacHelper SCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper SCH80211p = Wifi80211pHelper::Default ();
	NqosWaveMacHelper CCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper CCH80211p = Wifi80211pHelper::Default ();

	//data mode:The transmission mode to use for every data packet transmission
	//control mode: The transmission mode to use for every RTS packet transmission. here control mode is only 3Mbps
	SCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
										"DataMode",StringValue (phyMode),
										"ControlMode",StringValue ("OfdmRate3MbpsBW10MHz"));
	CCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
											"DataMode",StringValue (phyMode),
											"ControlMode",StringValue ("OfdmRate3MbpsBW10MHz"));

	// Set Tx Power For The SCH
	m_SCHPhy.Set ("TxPowerStart",DoubleValue (txp1));
	m_SCHPhy.Set ("TxPowerEnd", DoubleValue (txp1));
	m_SCHDevices = SCH80211p.Install(m_SCHPhy, SCH80211pMac, m_nodes);

	// Set Tx Power For The CCH
	m_CCHPhy.Set ("TxPowerStart",DoubleValue (txp2));
	m_CCHPhy.Set ("TxPowerEnd", DoubleValue (txp2));
	m_CCHDevices = CCH80211p.Install(m_CCHPhy, CCH80211pMac, m_nodes);

}

void VanetSim::ConfigDevices()
{
	//Done in ConfigChannels()
}

void VanetSim::ConfigMobility()
{
/*	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install (m_nodes.Begin(),m_nodes.End()-3);
	// configure movements for Car node, while reading trace file
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum);//Controller
	Temp->SetPosition(Vector(0.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+1);//source
	Temp->SetPosition(Vector(5.1, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+2);//Sink
*/
	VMo->Install();
	//std::cout<<"ConfigMobility: "<<std::endl;
	double rt = VMo->GetReadTotalTime();
	if (duration<0)
	{
		duration = rt;
	}
	Time temp_now = Simulator::Now();
	 std::cout<<"Now?"<<temp_now.GetSeconds ()<<std::endl;

	//add fixed node
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum)->GetObject<MobilityModel>();	//LC1
	Temp->SetPosition(Vector(500.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 1)->GetObject<MobilityModel>();	//LC2
	Temp->SetPosition(Vector(1000.0, 500.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 2)->GetObject<MobilityModel>();	//LC3
	Temp->SetPosition(Vector(1500.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 3)->GetObject<MobilityModel>();	//LC4
	Temp->SetPosition(Vector(1000.0, 1500.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 4)->GetObject<MobilityModel>();	//Source1
	Temp->SetPosition(Vector(-150.0, 998.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 5)->GetObject<MobilityModel>();	//Source2
	Temp->SetPosition(Vector(-150.0, 999.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 6)->GetObject<MobilityModel>();	//Source3
	Temp->SetPosition(Vector(-150.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 7)->GetObject<MobilityModel>();	//Sink1
	Temp->SetPosition(Vector(1000.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 8)->GetObject<MobilityModel>();	//Sink2
	Temp->SetPosition(Vector(1001.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 9)->GetObject<MobilityModel>();	//Sink3
	Temp->SetPosition(Vector(1002.0, 1000.0, 0.0));
/*	Temp = m_nodes.Get(nodeNum + 10)->GetObject<MobilityModel>();	//Sink4
	Temp->SetPosition(Vector(1000.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 11)->GetObject<MobilityModel>();	//Sink5
	Temp->SetPosition(Vector(2000.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum + 12)->GetObject<MobilityModel>();	//Sink6
	Temp->SetPosition(Vector(1000.0, 2000.0, 0.0));*/
}

void VanetSim::ConfigApp()
{
	//===Routing
	InternetStackHelper internet;
	OlsrHelper olsr;
	SdnHelper sdn;
	AodvHelper aodv;
	DsdvHelper dsdv;
	DsrHelper dsr;
	DsrMainHelper dsrMain;
	switch (mod)
	  {
      case 0:
        internet.SetRoutingHelper(olsr);
        internet.Install (m_nodes);
         std::cout<<"OLSR"<<std::endl;
         os<<"OLSR"<<std::endl;
        break;
      case 2:
        internet.SetRoutingHelper(aodv);
        internet.Install (m_nodes);
         std::cout<<"AODV"<<std::endl;
         os<<"AODV"<<std::endl;
        break;
      case 3:
        internet.SetRoutingHelper(dsdv);
        internet.Install (m_nodes);
         std::cout<<"DSDV"<<std::endl;
         os<<"DSDV"<<std::endl;
        break;
      case 4:
        internet.Install (m_nodes);
        dsrMain.Install (dsr, m_nodes);
         std::cout<<"DSRMain"<<std::endl;
        os<<"DSRMain"<<std::endl;
        break;
      default:
        for (uint32_t i = 0; i<nodeNum; ++i)
        {
            sdn.SetNodeTypeMap (m_nodes.Get (i), sdn::CAR);
        }

        for (uint32_t i = 0; i<numofLC; ++i)
        {
            sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+i), sdn::LOCAL_CONTROLLER);
        }
        for (uint32_t i = 0; i<numofSource; ++i)
        {
            sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+numofLC+i), sdn::OTHERS);
        }
        for (uint32_t i = 0; i<numofSink; ++i)
        {
            sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+numofLC+numofSource+i), sdn::OTHERS);
        }

        sdn.SetSR (range1);
        internet.SetRoutingHelper(sdn);
        internet.Install (m_nodes);
         std::cout<<"SetRoutingHelper Done"<<std::endl;
	  }

	std::cout<<"internet.Install Done"<<std::endl;

	//===IP ADDRESS
	Ipv4AddressHelper ipv4S;
	NS_LOG_INFO ("Assign IP Addresses.");
	ipv4S.SetBase ("10.1.0.0", "255.255.0.0");//SCH 256*256
	m_SCHInterfaces = ipv4S.Assign (m_SCHDevices);
	 std::cout<<"IPV4S Assigned"<<std::endl;

	Ipv4AddressHelper ipv4C;
	if ((mod ==1)||(mod ==5))
	{
		NS_LOG_INFO ("Assign IP-C Addresses.");
		ipv4C.SetBase("192.168.0.0","255.255.0.0");//CCH 256*256
		m_CCHInterfaces = ipv4C.Assign(m_CCHDevices);
		 std::cout<<"IPV4C Assigned"<<std::endl;
		for (uint32_t i = 0;i<m_nodes.GetN ();++i)
		{
		     //std::cout<<"m_nodes.GetN () "<<i<<std::endl;
		    Ptr<sdn::RoutingProtocol> routing = m_nodes.Get (i)->GetObject<sdn::RoutingProtocol> ();
		    routing->SetCCHInterface (m_CCHInterfaces.Get (i).second);
		    routing->SetSCHInterface (m_SCHInterfaces.Get (i).second);
		}

		sdn::Algo ToBeSet = sdn::Binary_Search;
		if (mod == 5)
		{
		    ToBeSet = sdn::Yangs_Algo;
		}

		// change setcontrollArea
		Ptr<sdn::RoutingProtocol> routing = m_nodes.Get (nodeNum)->GetObject<sdn::RoutingProtocol> ();
		routing->SetControllArea (Vector2D (0,990), Vector2D (1000,1000));// 990,1000 - 1000,1000
		routing->SetAlgo (ToBeSet);
		routing = m_nodes.Get (nodeNum+1)->GetObject<sdn::RoutingProtocol> ();
		routing->SetControllArea (Vector2D (990,990), Vector2D (1000,0));//
		routing->SetAlgo (ToBeSet);
		routing = m_nodes.Get (nodeNum+2)->GetObject<sdn::RoutingProtocol> ();
		routing->SetControllArea (Vector2D (1000,990), Vector2D (2000,1000));//todo 1010,990 - 1000,990
		routing->SetAlgo (ToBeSet);
		routing = m_nodes.Get (nodeNum+3)->GetObject<sdn::RoutingProtocol> ();
		routing->SetControllArea (Vector2D (1000,1000), Vector2D (1010,2000));
		routing->SetAlgo (ToBeSet);
	}

	//todo see wifi-hiddden-terminal
	/** \internal
	   * We also use separate UDP applications that will send a single
	   * packet before the CBR flows start.
	   * This is a workaround for the lack of perfect ARP, see \bugid{187}
	   */
	//===Traffic
	//source
	//onoff
	Address remote1;
	Address remote2;
	Address remote3;
	//transmittion method
	if ((mod == 1)||(mod==5))
	  {
		//brocast in different port

	    for(int i=0;i<numofSource;i++)
	    {
	    	Address remote;
		    std::pair<Ptr<Ipv4>, uint32_t> RetValue = m_SCHInterfaces.Get (nodeNum+numofLC+i);
		    Ipv4InterfaceAddress theinterface = RetValue.first->GetAddress (RetValue.second, 0);
		    Ipv4Address bcast = theinterface.GetLocal ().GetSubnetDirectedBroadcast (theinterface.GetMask ());
		    remote = InetSocketAddress(bcast, m_port[i]);
			OnOffHelper Source("ns3::UdpSocketFactory",remote);//SendToSink--Brocast

			Source.SetConstantRate(DataRate("10kbps"));
			//set data rate

			//set ontime:duration of ontime will send data
			Source.SetAttribute("OnTime",StringValue ("ns3::ConstantRandomVariable[Constant=1]"));

			//set offtime:duration of offtime willnot send data
			//Source.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

			//set packet size
			Source.SetAttribute("PacketSize", UintegerValue (packetSize));
			//install on node and set start time and stop time
			m_source = Source.Install(m_nodes.Get(nodeNum+numofLC+i));//Insatll on Source1
			m_source.Start(Seconds(140.0+i*0.01));
			m_source.Stop(Seconds(250.0+i*0.01));//Default Start time is 0.

		    //statistics:trace function recall
			std::string temp1 = "/NodeList/"+std::to_string (nodeNum+numofLC+i)+"/ApplicationList/0/$ns3::OnOffApplication/Tx";
			Config::Connect (temp1,MakeCallback(&VanetSim::TXTraces, this));//todo 这有什么作用?回调函数txtrace

			//sink
			TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
			Ptr<Socket> sink = Socket::CreateSocket (m_nodes.Get(nodeNum+numofLC+numofSource+i), tid);//The Sink1
			//HearALL;
			InetSocketAddress local = InetSocketAddress(Ipv4Address::GetZero (),m_port[i]);
			sink->Bind(local);
			sink->SetRecvCallback(MakeCallback(&VanetSim::ReceiveDataPackets, this));
			//sink->SetAttribute("num", StringValue(i));
	    }

	  }
	else
	  {
		//directly send to destination in different port
//	    remote1 = InetSocketAddress(m_SCHInterfaces.GetAddress (nodeNum+numofLC+numofSource), m_port1);
//	    remote2 = InetSocketAddress(m_SCHInterfaces.GetAddress (nodeNum+numofLC+numofSource+1), m_port2);
//	    remote3 = InetSocketAddress(m_SCHInterfaces.GetAddress (nodeNum+numofLC+numofSource+2), m_port3);
	  }
}

void VanetSim::ReceiveDataPackets(Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	while ((packet = socket->Recv()))
	{
		uint64_t uid = packet->GetUid();
		//no udheader
		Address to;
		socket->GetSockName(to);
		InetSocketAddress address = InetSocketAddress::ConvertFrom (to);
		int num= address.GetPort() - base_port;

		std::cout<<num<<std::endl;
		if (statistics[num].dup_det.find(uid) == statistics[num].dup_det.end())
		{
			statistics[num].Unique_RX_Pkts++;
			statistics[num].dup_det.insert(uid);

			Time now = Simulator::Now();
			int64_t temp = now.GetMicroSeconds() - statistics[num].delay[uid].GetMicroSeconds();
			statistics[num].delay_vector.push_back(temp);
			statistics[num].per_sec_delay_vector.push_back(temp);
		}
		//Rx_Data_Bytes += packet->GetSize();
		statistics[num].Rx_Data_Pkts++;
		std::cout <<num<< "." << std::flush;
	}
}

void VanetSim::SendDataPacket()
{
	/*Ptr<Packet> packet = Create<Packet> (packetSize);
	source->SendTo(packet, 0, )
	Simulator::Schedule(Seconds(interval), &VanetSim::SendDataPacket, this);*/
	//TODO
}

void VanetSim::ConfigTracing()
{
	//TODO
	//done in configapp
}

void VanetSim::ProcessOutputs()
{
	for(int i=0;i<numofSource;i++)
	{
		std::cout << "Source" << i << ":" << std::endl;
		std::cout << "	Tx_Data_Pkts:   " << statistics[i].Tx_Data_Pkts << std::endl;
		std::cout << "	Rx_Data_Pkts:   " << statistics[i].Rx_Data_Pkts << std::endl;
		std::cout << "	Unique_RX_Pkts: " << statistics[i].Unique_RX_Pkts << std::endl;

		if (statistics[i].Tx_Data_Pkts != 0)
			std::cout << "Reception Rate "<< i << ":"
					<< ((double) statistics[i].Unique_RX_Pkts / statistics[i].Tx_Data_Pkts ) * 100
					<< std::endl;
		if (!statistics[i].delay_vector.empty())
		{
			int64_t best = statistics[i].delay_vector[0], worst = statistics[i].delay_vector[0];
			double avg = 0;
			for (std::vector<int64_t>::const_iterator cit = statistics[i].delay_vector.begin();
					cit != statistics[i].delay_vector.end(); ++cit)
			{
				if (*cit < best)
				{
					best = *cit;
				}

				if (*cit > worst)
				{
					worst = *cit;
				}
				avg += *cit;
			}

			avg /= statistics[i].delay_vector.size();
			std::cout << "Best delay:   " << best << "us" << std::endl;
			std::cout << "Worst delay:   " << worst << "us" << std::endl;
			std::cout << "Avg delay: " << avg << "us" << std::endl;
			os << "Best delay:   " << best << "us" << std::endl;
			os << "Worst delay:   " << worst << "us" << std::endl;
			os << "Avg delay: " << avg << "us" << std::endl;
		}
		else
		{
			std::cout << "NO PACKETS WERE RECEIVED." << std::endl;
			os << "NO PACKETS WERE RECEIVED." << std::endl;
			dos << "NO PACKETS WERE RECEIVED." << std::endl;
		}
	}
}

void VanetSim::Run()
{
	Simulator::Schedule(Seconds(0.0), &VanetSim::Look_at_clock, this);
	std::cout << "Starting simulation for " << duration << " s ..."<< std::endl;
	os << "Starting simulation for " << duration << " s ..."<< std::endl;
	Simulator::Stop(Seconds(duration));
	Simulator::Run();
	VanetSim::statistic();

	Simulator::Destroy();
}

void VanetSim::statistic()
{
	//lc
	Ptr<sdn::RoutingProtocol> routing;	// get statistic data
	for (int i = 0; i < 4; i++)
	{
		routing = m_nodes.Get(nodeNum + i)->GetObject<sdn::RoutingProtocol>();
		m_numofapp += routing->m_numofapp;
		m_numofackhello += routing->m_numofackhello;
		m_numofdontforward += routing->m_numofdontforward;
		m_numoflc2lc += routing->m_numoflc2lc;
		m_numofallforwardcar += routing->m_allforwardcar.size();
	}
	//car
	for (int i = 0; i < nodeNum; i++)
	{
		routing = m_nodes.Get(i)->GetObject<sdn::RoutingProtocol>();
		//m_numofhm += routing->m_numofhm;
		m_numofhm = (250 - 140) / 0.5;	//routing->m_helloInterval
		m_numofdatapacket += routing->m_numofdatapacket;
	}

	std::cout << "hm:" << m_numofhm << "   app:" << m_numofapp << "  ackh: "
			<< m_numofackhello << "   dontforward:" << m_numofdontforward
			<< "   lc2lc:" << m_numoflc2lc << "   datapack:"
			<< m_numofdatapacket << "   allforwardcar:" << m_numofallforwardcar
			<< "   " << std::endl;
}

void VanetSim::Look_at_clock()
{
	std::cout << "Now:" << Simulator::Now().GetSeconds();
	std::cout << "  Mode: " << m_todo << "  ,DataSet: " << m_ds << std::endl;

	for(int i=0;i<numofSource;i++)
	{
		std::cout << "Source" << i << ":" << std::endl;
		std::cout << "	Tx_Data_Pkts:   " << statistics[i].Tx_Data_Pkts << std::endl;
		std::cout << "	Rx_Data_Pkts:   " << statistics[i].Rx_Data_Pkts << std::endl;
		std::cout << "	Unique_RX_Pkts: " << statistics[i].Unique_RX_Pkts << std::endl;

		if (statistics[i].Tx_Data_Pkts != 0)
			std::cout << "Reception Rate "<< i << ":"
					<< ((double) statistics[i].Unique_RX_Pkts / statistics[i].Tx_Data_Pkts ) * 100
					<< std::endl;
		statistics[i].old_Unique_RX_Pkts = statistics[i].Unique_RX_Pkts;
		statistics[i].old_Rx_Data_Pkts = statistics[i].Rx_Data_Pkts;
		statistics[i].old_Unique_RX_Pkts = statistics[i].Unique_RX_Pkts;
		statistics[i].old_Rx_Data_Pkts = statistics[i].Rx_Data_Pkts;
		statistics[i].old_Tx_Data_Pkts = statistics[i].Tx_Data_Pkts;
	}

	Simulator::Schedule(Seconds(1.0), &VanetSim::Look_at_clock, this);
}

void VanetSim::TXTraces(std::string context,Ptr<const Packet> newpacket)
{
	//packetsize = 1000,means no useful information
	int noofnode =0;
	int pcontext=10;
	while(context[pcontext]!='/')
	{
		noofnode = noofnode*10 + context[pcontext] -'0';
		pcontext++;
	}

	int num = noofnode - (nodeNum+numofLC);
	statistics[num].Tx_Data_Pkts++;
	statistics[num].Tx_Data_Bytes += newpacket->GetSize();
	//std::cout<<"ANOTHER ONE!HAHAHA"<<std::endl;

	Time now = Simulator::Now();
	statistics[num].delay[newpacket->GetUid()] = now;
}

// Example to use ns2 traces file in ns3
int main(int argc, char *argv[])
{
	//LogComponentEnable("MacLow",LOG_LEVEL_ALL);
	VanetSim SDN_test;
	SDN_test.Simulate(argc, argv);
	return 0;
}



