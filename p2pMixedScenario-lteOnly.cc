/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Jaume Nin <jaume.nin@cttc.cat>
 */

#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/wifi-module.h"
#include "ns3/csma-module.h"
#include "ns3/bridge-helper.h"
//#include "ns3/gtk-config-store.h"

#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"

#include <vector>
#include <numeric>
#include <ns3/assert.h>

using namespace ns3;

/**
 * Sample simulation script for LTE+EPC. It instantiates one eNodeB,
 * attaches one or various UE to said eNodeB, attaches (a) wifi station(s) to a wifi AP interface at
 * the UE, and starts a flow for each wifi station to a remote host via AP and LTE+EPC.
 * Author: Luis Sanabria-Russo
 * luis.sanabria@cttc.es
 */

NS_LOG_COMPONENT_DEFINE ("p2pMixedScenario");

struct metrics
{
  std::map <uint32_t, std::map <uint32_t, double> > numRxPackets;
  std::map < uint32_t, std::map <uint32_t, double> > numGenPackets;
  std::map <uint32_t, std::map <uint32_t, std::vector<uint64_t> > > delay;
  double genBytes;
  double rxBytes;
};

struct packetTracer
{
  std::map < uint32_t, std::vector < std::map < uint64_t, Time > > > map;
  std::map < uint32_t, std::map < Ipv4Address, uint32_t > > ids;
};

void
positionNodeInsideDisk (uint32_t cluster, double d, MobilityHelper mobility, Ptr<Node> node)
{ 
  Vector position;
  double radius = d / 10.0;
  std::string theta = "ns3::UniformRandomVariable[Min=0|Max=6.2830]"; // the whole disk
  std::ostringstream oss3;
  oss3 << "ns3::UniformRandomVariable[Min=0.0" << "|Max=" << radius << "]";
  std::string rho = oss3.str();

  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                  "X", DoubleValue (cluster * d),
                                  "Y", DoubleValue (0.0),
                                  "Theta", StringValue (theta),
                                  "Rho", StringValue (rho));
  mobility.Install (node);
}

void
writeOverallResultsToFile (uint32_t clusters, std::vector<double> &throughput, 
  std::vector<double> &delay, std::vector<double> &pdr, double &genRate, 
  std::string filename, uint32_t uePerCluster)
{ 
  double avg_t = (std::accumulate (throughput.begin (), throughput.end (), 0.0));

  double avg_d = (std::accumulate (delay.begin (), delay.end (), 0.0))
          / delay.size () * 1.0;

  double avg_p = (std::accumulate (pdr.begin (), pdr.end (), 0.0))
          / pdr.size () * 1.0;


  std::ofstream file;
  file.open (filename, std::ofstream::out | std::ofstream::app);
  file << clusters << " " << avg_t << " " << avg_d << " " 
        << avg_p  << " " << genRate << " " << uePerCluster << "\n";
  file.close ();
}

Vector
getNodePosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mm = node->GetObject<MobilityModel> ();
  return (mm->GetPosition ());
}

std::tuple<uint32_t, uint32_t>
getAssociatedClusterForAddress (struct packetTracer *tracer, const Ipv4Address &address)
{
  uint32_t cluster;
  uint32_t id;
  Ipv4Address srcAddr =  address;

  for (std::map <uint32_t, std::map <Ipv4Address, uint32_t> >::iterator i = tracer->ids.begin ();
    i != tracer->ids.end (); ++i )
    { 
      cluster = i->first;
      if (i->second.find (srcAddr) != i->second.end ())
        { 
          id = i->second.find (srcAddr)->second;
          break;
        }
    }

  return std::make_tuple(cluster, id);
}

// Trace Callbacks

static void
TraceRxPackets(struct metrics *res, struct packetTracer *tracer, 
  Ptr< const Packet > packet, const Ipv4Address &address)
{
  
  Time genTime;
  uint32_t id;
  uint32_t cluster;

  std::tie(cluster, id) = getAssociatedClusterForAddress (tracer, address);

  SeqTsHeader seqTs;
  packet->PeekHeader (seqTs);
  uint64_t uId = seqTs.GetSeq ();

  std::vector< std::map< uint64_t, Time > > genPkts = tracer->map[id];
  uint64_t pos = 0;
  for (std::vector< std::map< uint64_t, Time > >::iterator i = genPkts.begin();
    i != genPkts.end(); ++i)
    { 
      std::map< uint64_t, Time > j = *i;
      if (j.count (uId) > 0) // if it exists
        {
          genTime = j[uId];
          tracer->map[id].erase(tracer->map[id].begin() + pos);
          break;
        }
      else
        { 
          continue;
        }
      ++pos;
    }
  NS_ASSERT (!genTime.IsZero ());

  Time d = Simulator::Now () - genTime;
  res->numRxPackets[cluster][id]++;
  res->delay[cluster][id].push_back(d.GetMicroSeconds ());
  res->rxBytes += double (packet->GetSize ());
  
  /*std::cout << "--->Packet received from: " << address << ", uId: " << uId << std::endl;
  std:: cout << "\t--->Delay suffered by packet: " << d.GetMicroSeconds () << " us" << std::endl << std::endl;*/
  
}


static void
TraceGenPackets(struct metrics *res, struct packetTracer *tracer, std::string context,
  Ptr< const Packet > packet, const uint32_t &seqNum, const uint32_t &node, Ptr<const Node> sender)
{
  
  uint32_t id;
  uint32_t cluster;
  Ipv4Address senderIp = sender->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();

  std::tie (cluster, id) = getAssociatedClusterForAddress (tracer, senderIp);

  res->numGenPackets[cluster][id]++;

  std::map< uint64_t, Time> m;
  m[seqNum] = Simulator::Now ();

  tracer->map[node].push_back(m);
  res->genBytes += double (packet->GetSize ());

  
  /*std::cout << "--->Packet generated by: " << senderIp << 
    ", seq: " << seqNum << std::endl;*/ 
}


int
main (int argc, char *argv[])
{
  uint32_t seed = 0;

  // LTE+EPC
  uint16_t numberOfLteNodes = 3;
  uint16_t numberOfUeClusters = 2;
  double simTime = 5.1;
  double distance = 60.0;

  // Traffic
  uint32_t mtu = 1500;
  std::string genericMmsDataRate = "0.008Mbps"; // Application layer datarate

  // metrics
  struct metrics results;
  struct packetTracer tracer;
  std::string filename = "results-p2pMixedScenario-lteOnly.log";

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("numberOfLteNodes", "Number of UE", numberOfLteNodes);
  cmd.AddValue ("numberOfUeClusters", "Number of groups of UEs", numberOfUeClusters);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("distance", "Distance between UE [m]", distance);
  cmd.AddValue ("seed", "Enter a number greater than zero to change the seed", seed);
  cmd.Parse(argc, argv);

  /* Changing the simulation seed */
  if (seed > 0)
    RngSeedManager::SetSeed (seed);

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  // parse again so you can override default values from the command line
  cmd.Parse(argc, argv);

  Ptr<Node> pgw = epcHelper->GetPgwNode (); //get the gateway entity of the EPC

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (mtu));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Creating the LTE+EPC network
  NS_ASSERT (numberOfUeClusters > 0);
  NS_ASSERT (numberOfLteNodes > 0);

  NodeContainer ueNodes;
  NodeContainer enbNodes;
  std::vector<NodeContainer> perClusterUes;
  enbNodes.Create(1); // a single eNodeB

  // Install Mobility Model
  Ptr<ListPositionAllocator> possitionAlloc = CreateObject<ListPositionAllocator> ();
  possitionAlloc->Add (Vector (0.0, 0.0, 0.0));
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (possitionAlloc);
  mobility.Install(enbNodes);

  for (uint16_t c = 0; c < numberOfUeClusters; ++c)
    {
      NodeContainer ues;
      ues.Create (numberOfLteNodes);
      for (uint16_t u = 0; u < ues.GetN (); ++u)
        { 
          // under this scheme, the eNodeB is at position 0,0,0, where all
          // other UEs are randomly distributed inside a disk with radius distance/10 meters, 
          // centered at points separated by distance meters, forming a line

          positionNodeInsideDisk (c, distance, mobility, ues.Get (u));
        }
      ueNodes.Add (ues);
      perClusterUes.push_back (ues);
    }

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);

  NetDeviceContainer ueLteDevs;
  std::vector<NetDeviceContainer> perClusterUeLteDevs;
  for (uint16_t c = 0; c < perClusterUes.size (); ++c)
    { 
      NetDeviceContainer dev = lteHelper->InstallUeDevice (perClusterUes.at (c));
      ueLteDevs.Add (dev);
      perClusterUeLteDevs.push_back (dev);
    }

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  std::vector<Ipv4InterfaceContainer> perClusterUeIpv4Interfaces;
  for (uint16_t c = 0; c < perClusterUes.size (); ++c)
    { 
      Ipv4InterfaceContainer iface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (perClusterUeLteDevs.at (c)));
      ueIpIface.Add (iface);
      perClusterUeIpv4Interfaces.push_back (iface);
    }

  // Define a default route through pgw
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      // Attach UEs to the eNodeB
      lteHelper->Attach (ueLteDevs.Get (u), enbLteDevs.Get (0));
    }
  
  // Increasing SrsPeriodicity in eNodeB to its maximum value, 320 ms.
  // This allows simulation scenarios for more than 20 UE
  if (ueNodes.GetN () >= 20)
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice/LteEnbRrc/SrsPeriodicity", UintegerValue (320));

 
  // Install and start applications on UEs and remote host 
  uint16_t ulPort = 2000;
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  std::vector<ApplicationContainer> perClusterServerApps;
  uint32_t payloadSize = 1472; //Bytes
  Ptr<FlowMonitor> monitor;
  FlowMonitorHelper flowHelper;

  for (uint32_t c = 0; c < perClusterUes.size (); ++c)
    { 
      ApplicationContainer clusterServerApp;
      for (uint32_t u = 0; u < perClusterUes.at (c).GetN (); ++u)
        {
          ++ulPort;          
          payloadSize = 57; // loadsize of MMS response segment
          std::cout <<"Simulation: OnOff UDP generated at LTE UEs" << std::endl;

          // server
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          clusterServerApp.Add (ulPacketSinkHelper.Install (remoteHost));

          // Saving information from all wifiStations for the trace callbacks
          Ptr<Node> ue = perClusterUes.at (c).Get (u);
          Ptr<Ipv4> ip = ue->GetObject<Ipv4> ();
          Ipv4Address addr = ip->GetAddress (1,0).GetLocal ();
          tracer.ids[c][addr] = ue->GetId ();

          // UE transmitters
          OnOffHelper ulClient ("ns3::UdpSocketFactory", (InetSocketAddress (remoteHostAddr, ulPort)));
          ulClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));
          ulClient.SetAttribute ("OnTime", StringValue ("ns3::UniformRandomVariable"));
          ulClient.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
          ulClient.SetAttribute ("DataRate", DataRateValue (DataRate (genericMmsDataRate)));

          clientApps.Add (ulClient.Install (perClusterUes.at (c).Get(u)));
        }
      perClusterServerApps.push_back (clusterServerApp);
      serverApps.Add (clusterServerApp);
    }
  serverApps.Start (Seconds (0.5));
  clientApps.Start (Seconds (1.0));

  //lteHelper->EnableTraces (); // outputs .txt files 
  // Uncomment to enable PCAP tracing
  //p2ph.EnablePcapAll("lena-epc-first");


  // Checking IPs
  std::cout << "LTE+EPC network specs" << std::endl;
  std::cout << "=====================" << std::endl,
  std::cout << "-RemoteHost: " << remoteHostAddr << std::endl;
  std::cout << "-pgw: " << internetIpIfaces.GetAddress (0) << std::endl;
  std::cout << "-eNodeB: " << epcHelper->GetUeDefaultGatewayAddress () << std::endl;

  for (uint32_t c = 0; c < perClusterUeIpv4Interfaces.size (); ++c)
    { 
      std::cout << "\n-Cluster-" << c << std::endl;
      for (uint32_t h = 0; h < perClusterUeIpv4Interfaces.at (c).GetN (); ++h)
        { 
          std::cout << "\t-UE-" << h << ": " << perClusterUeIpv4Interfaces.at (c).GetAddress (h) << std::endl;
        }
    }
  
  std::cout << std::endl;

  // Checking positions
  std::cout << "Mobility details" << std::endl;
  std::cout << "================" << std::endl;
  for (uint32_t c = 0; c < perClusterUes.size (); ++c)
    { 
      std::cout << "-Cluster-" << c << std::endl;
      for (uint32_t l = 0; l < perClusterUes.at (c).GetN (); ++l)
        { 
          std::cout << "\t-UE-" << l << ": ";
          Vector pos = Vector (getNodePosition (perClusterUes.at (c).Get (l)));
          std::cout << "X: " << pos.x << ", Y: " << pos.y << ", Z: " << pos.z << std::endl;
        }
    }

  // Connecting trace sources
  Config::ConnectWithoutContext ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
                  MakeBoundCallback(&TraceRxPackets, &results, &tracer));

  Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",
                  MakeBoundCallback(&TraceGenPackets, &results, &tracer));


  Simulator::Stop (Seconds (simTime+1.0));
  Simulator::Run();

  /*GtkConfigStore config;
  config.ConfigureAttributes();*/

  Ptr<PacketSink> sink;
  double throughput;
  double avg_delay;
  double pdr;

  std::vector<double> avgThroughput;
  std::vector<double> overallDelay;
  std::vector<double> averagePdr;

  std::cout << std::endl;
  std::cout << "Simulation metrics" << std::endl;
  std::cout << "==================" << std::endl;

  for (uint32_t c = 0; c < perClusterServerApps.size (); ++c)
    { 
      std::cout << "-Cluster-" << c << std::endl;
      for (uint32_t a = 0; a < perClusterServerApps.at (c).GetN (); ++a)
        { 

          Ptr<Node> ue = perClusterUes.at (c).Get (a);
          uint32_t id = ue->GetId ();
          sink = StaticCast<PacketSink> (perClusterServerApps.at (c).Get (a));
          throughput = ((sink->GetTotalRx () * 8) / (1e6  * simTime));
          avg_delay = (std::accumulate (results.delay[c][id].begin (), results.delay[c][id].end (), 0.0)) 
            / results.delay[c][id].size () * 1.0;
          pdr = results.numRxPackets[c][id] / results.numGenPackets[c][id];

          std::cout << "\tUE-" << a << ")--->Throughput : " << throughput << " Mbps, ";
          std::cout << "\tDelay: " << avg_delay << " us, ";
          std::cout << "\tPDR: " << pdr << "/1" << std::endl;

          avgThroughput.push_back (throughput);
          overallDelay.push_back (avg_delay);
          averagePdr.push_back (pdr);
        }
    }
  
  double relativeService = results.rxBytes / results.genBytes;
  double generationRate = results.genBytes * 8 / (1e6 * simTime);
  std::cout << std::endl << "--->Overall: Throughput (" << results.rxBytes * 8 / (1e6 * simTime) << 
    " Mbps) / Generation rate (" << generationRate << " Mbps) : " << 
    relativeService << "/1" << std::endl;
  
  writeOverallResultsToFile (numberOfUeClusters, avgThroughput, overallDelay, averagePdr, 
    generationRate, filename, numberOfLteNodes);

  Simulator::Destroy();
  return 0;

}