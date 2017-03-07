/*
 * SDN.h
 *
 *  Created on: Oct 13, 2015
 *      Author: chl
 */

#ifndef SDN_H
#define SDN_H

#include <string>
#include <fstream>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"

//FINALLY!
#include "ns3/sdn-helper.h"
#include "ns3/vanetmobility-helper.h"

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "ns3/athstats-helper.h"

using namespace ns3;

class transmissionStatistics
{
public:
	uint32_t Rx_Data_Bytes, Tx_Data_Bytes;
	uint32_t Rx_Data_Pkts, Tx_Data_Pkts;
	uint32_t old_Tx_Data_Pkts,old_Rx_Data_Pkts;
	uint32_t Unique_RX_Pkts, old_Unique_RX_Pkts;

	std::unordered_set<uint64_t> dup_det;
	std::unordered_map<uint64_t, Time> delay;
	std::vector<int64_t> delay_vector;
	std::vector<int64_t> per_sec_delay_vector;
};

class VanetSim
{
public:
	VanetSim();
	~VanetSim();
	void Simulate(int argc, char *argv[]);
protected:
	void SetDefault();
	void ParseArguments(int argc, char *argv[]);
	void LoadTraffic();
	void ConfigNode();
	void ConfigChannels();
	void ConfigDevices();
	void ConfigMobility();
	void ConfigApp();
	void ConfigTracing();
	void Run();
	void statistic();
	void ProcessOutputs();
	bool CheckActive(Node node);
	void Look_at_clock();
private:
	Ptr<Socket> source;
	std::string traceFile;
	std::string logFile;
	std::string phyMode;
	std::string lossModle;
	std::string delayModle;
	std::string homepath;
	std::string folder;
	std::ofstream os;
	std::ofstream dos; //For Delay File
	double freq1; //SCH
	double freq2; //CCH
	double txp1; //SCH
	double txp2; //CCH
	double range1; //SCH
	double range2; //CCH
	uint32_t packetSize; // bytes
	//uint32_t numPackets;
	//double interval; // seconds
	bool verbose;
	int mod;	//1=SDN Other=OLSR
	uint32_t nodeNum;
	int numofLC, numofSource, numofSink;
	double duration;
	YansWifiPhyHelper m_SCHPhy, m_CCHPhy;
	NodeContainer m_nodes;	//Cars + Controller + Source + Sink
	NetDeviceContainer m_SCHDevices, m_CCHDevices;
	Ipv4InterfaceContainer m_SCHInterfaces, m_CCHInterfaces;
	//////////TongJi////////////
	ApplicationContainer m_source;
	Ptr<ns3::vanetmobility::VANETmobility> VMo;
	void ReceiveDataPackets(Ptr<Socket> socket);
	void SendDataPacket();
	void TXTraces(std::string context,Ptr<const Packet> newpacket);
	std::vector<transmissionStatistics> statistics;
	std::vector<uint64_t> m_port;
	uint32_t base_port;

	std::string m_todo;
	std::string m_ds;	//DataSet

	int m_numofhm, m_numofapp, m_numofackhello, m_numofdontforward,
			m_numoflc2lc;
	int m_numofdatapacket, m_numofallforwardcar;
	std::vector<Ipv4Address> m_allforwardcar;

};

#endif /* SDN_H */
