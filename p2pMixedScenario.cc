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
  std::map < uint32_t, double> numRxPackets;
  std::map <uint32_t, double> numGenPackets;
  std::map < uint32_t, std::vector<uint64_t> > delay;
  double genBytes;
  double rxBytes;
};

struct packetTracer
{
  std::map < uint32_t, std::vector < std::map < uint64_t, Time > > > map;
  std::map < uint32_t, std::map < Ipv4Address, uint32_t > > ids;
};

void
writePerApMetrics (uint32_t a, double throughput, double avg_delay, double pdr)
{
  std::string filename = "results-perAp-p2pMixedScenario.log";
  std::ofstream file;
  file.open (filename, std::ofstream::out | std::ofstream::app);
  file << a << " " << throughput << " " << avg_delay << " " << pdr << "\n";
  file.close ();
}

void
writeOverallResultsToFile (uint32_t servers, std::vector<double> &throughput,
  std::vector<double> &delay, std::vector<double> &pdr, double &genRate, std::string filename,
  uint16_t &wifiNodes)
{
  double avg_t = (std::accumulate (throughput.begin (), throughput.end (), 0.0));

  double avg_d = (std::accumulate (delay.begin (), delay.end (), 0.0))
          / delay.size () * 1.0;

  double avg_p = (std::accumulate (pdr.begin (), pdr.end (), 0.0))
          / pdr.size () * 1.0;


  //std::string filename = "results-p2pMixedScenario.log";
  std::ofstream file;
  file.open (filename, std::ofstream::out | std::ofstream::app);
  file << servers << " " << avg_t << " " << avg_d << " " << avg_p  << " " << genRate << " " << wifiNodes << "\n";
  file.close ();
}

Vector
getNodePosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mm = node->GetObject<MobilityModel> ();
  return (mm->GetPosition ());
}

std::tuple<uint32_t, uint32_t>
getAssociatedApForAddress (struct packetTracer *tracer, const Ipv4Address &address)
{
  uint32_t ap;
  uint32_t id;
  Ipv4Address srcAddr =  address;

  for (std::map <uint32_t, std::map <Ipv4Address, uint32_t> >::iterator i = tracer->ids.begin ();
    i != tracer->ids.end (); ++i )
    {
      ap = i->first;
      if (i->second.find (srcAddr) != i->second.end ())
        {
          id = i->second.find (srcAddr)->second;
          break;
        }
    }

  return std::make_tuple(ap, id);
}

// Trace Callbacks

static void
TraceRxPackets(struct metrics *res, struct packetTracer *tracer,
  Ptr< const Packet > packet, const Ipv4Address &address)
{

  Time genTime;
  uint32_t id;
  uint32_t ap;

  std::tie(ap, id) = getAssociatedApForAddress (tracer, address);

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
  res->numRxPackets[ap]++;
  res->delay[ap].push_back(d.GetMicroSeconds ());
  res->rxBytes += double (packet->GetSize ());


  /*
  std::cout << std::endl << "--->Packet received from: " << InetSocketAddress::ConvertFrom(address).GetIpv4 ()
  << ", uId: " << uId << std::endl;
   std:: cout << "\t--->Delay suffered by packet: " << d.GetMicroSeconds () << " us" << std::endl;
  */
}


static void
TraceGenPackets(struct metrics *res, struct packetTracer *tracer, std::string context,
  Ptr< const Packet > packet, const uint32_t &seqNum, const uint32_t &node, Ptr<const Node> sender)
{

  uint32_t id;
  uint32_t ap;
  Ipv4Address senderIp = sender->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();

  std::tie (ap, id) = getAssociatedApForAddress (tracer, senderIp);

  res->numGenPackets[ap]++;

  std::map< uint64_t, Time> m;
  m[seqNum] = Simulator::Now ();

  tracer->map[node].push_back(m);
  res->genBytes += double (packet->GetSize ());

  /*
  std::cout << "--->Packet generated by NodeID: " << node <<
    ", seq: " << seqNum << std::endl;
  */

}


int
main (int argc, char *argv[])
{
  uint32_t seed = 0;

  // LTE+EPC
  uint16_t numberOfLteNodes = 3;
  double simTime = 5.1;
  double distance = 60.0;
  double interPacketInterval = 100;

  // Traffic
  uint32_t mtu = 1500;
  uint32_t OFF = 1;
  std::string genericMmsDataRate = "0.008Mbps"; // Application layer datarate
  bool wifiTraffic = true;
  bool remoteToWifi = true;
  bool genericMms = false; // if false = UDP source; true = TCP mimicking MMS
  bool enableFlowMon = false; // false leads to PCAP tracing
  bool udpBasic = false; // CBR UDP source

  // WiFi
  uint16_t numberOfWifiNodes = 5;
  uint32_t maxRetry = 7;
  uint32_t mcs = 7;
  uint32_t channelWidth = 20;
  double wifiRange = distance / 10.0; //using the default distance = 60m

  // metrics
  struct metrics results;
  struct packetTracer tracer;
  std::string filename = "results-p2pMixedScenario.log";

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("OFF", "Duration of OFF phase", OFF);
  cmd.AddValue ("numberOfLteNodes", "Number of UE", numberOfLteNodes);
  cmd.AddValue ("numberOfWifiNodes", "Number of WiFi nodes connected to UE's WiFi AP interface", numberOfWifiNodes);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("distance", "Distance between UE [m]", distance);
  cmd.AddValue ("interPacketInterval", "Inter packet interval [ms])", interPacketInterval);
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

  //Changing the results filename according to the number of wifiStations
  /*
  if (numberOfWifiNodes < 20)
    {
      filename = "results-p2pMixedScenario.log";
    }
  else if (numberOfWifiNodes >= 50 && numberOfWifiNodes < 75)
    {
      filename = "results-p2pMixedScenario-light.log";
    }
  else if (numberOfWifiNodes >= 75 && numberOfWifiNodes < 150)
    {
      filename = "results-p2pMixedScenario-medium.log";
    }
  else
    {
      filename = "results-p2pMixedScenario-congested.log";
    }
  */

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
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("192.168.0.0"), Ipv4Mask ("255.255.0.0"), 1);

  // Creating the LTE+EPC network
  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create(1); // a single eNodeB
  ueNodes.Create(numberOfLteNodes);

  // Install Mobility Model
  Ptr<ListPositionAllocator> possitionAlloc = CreateObject<ListPositionAllocator> ();
  possitionAlloc->Add (Vector (0.0, 0.0, 0.0));
  for (uint16_t i = 0; i < ueNodes.GetN (); ++i)
    {
      // under this scheme, the eNodeB is at position 0,0,0, where all
      // other UEs form a line on the X-axis
      possitionAlloc->Add (Vector(distance * (i + 1), 0, 0));
    }
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (possitionAlloc);
  mobility.Install(enbNodes);
  mobility.Install(ueNodes);

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  // Assign IP address to UEs and define a default route through pgw
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      // Attach UEs to the eNodeB
      lteHelper->Attach (ueLteDevs.Get (u), enbLteDevs.Get (0));
    }

  //lteHelper->ActivateDedicatedEpsBearer (ueLteDevs, EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT), EpcTft::Default ());

  // Increasing SrsPeriodicity in eNodeB to its maximum value, 320 ms.
  // This allows simulation scenarios for more than 20 UE
  if (ueNodes.GetN () >= 20)
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice/LteEnbRrc/SrsPeriodicity", UintegerValue (320));

  // Each UE has an ethernet interface which connects it to
  // a wireless router (Access Point). Here we create the CSMA link
  // between each UE and its respective wireless router.
  ipv4h.SetBase ("192.168.0.0", "255.255.0.0");

  NodeContainer wifiRouters;

  std::vector<Ipv4InterfaceContainer> lanIpIfaces;
  std::vector<NetDeviceContainer> lanDevices;

  wifiRouters.Create (ueNodes.GetN ());
  internet.Install (wifiRouters);
  Ptr<ListPositionAllocator> routerPossitionAlloc = CreateObject<ListPositionAllocator> ();
  NS_ASSERT (ueNodes.GetN () == wifiRouters.GetN ());
  for (uint32_t k = 0; k < ueNodes.GetN (); ++k)
    {
      NodeContainer lan (ueNodes.Get (k), wifiRouters.Get (k));

      CsmaHelper csma;
      csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
      csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (0.0)));

      NetDeviceContainer lanDev = csma.Install (lan); // device 0 is ue, 1 is router
      Ipv4InterfaceContainer lanIpIf = ipv4h.Assign (lanDev);

      // place each wifiRouter in the same place as the UE
      Ptr<MobilityModel> ueMm;
      Ptr<Node> ueNode = ueNodes.Get (k);
      ueMm = ueNode->GetObject<MobilityModel> ();
      NS_ASSERT (ueMm != 0);
      Vector uePos = ueMm->GetPosition ();
      routerPossitionAlloc->Add (uePos);

      // saving all in containers
      lanDevices.push_back (lanDev);
      lanIpIfaces.push_back (lanIpIf);

      // Route to get to the CSMA link of the UE
      Ptr<Node> ue = ueNodes.Get (k);
      Ptr<Ipv4StaticRouting> ueRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueRouting->AddNetworkRouteTo (Ipv4Address (ue->GetObject<Ipv4> ()->GetAddress(2, 0).GetLocal ().CombineMask (Ipv4Mask ("255.255.0.0"))),
        Ipv4Mask ("255.255.0.0"), 2);

      ipv4h.NewNetwork ();
    }
  mobility.SetPositionAllocator (routerPossitionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiRouters);


  // Defining default routes for the router
  for (uint32_t r = 0; r < ueNodes.GetN (); ++r)
    {
      Ptr<Node> router = wifiRouters.Get (r);
      Ptr<Ipv4StaticRouting> routerStaticRouting = ipv4RoutingHelper.GetStaticRouting (router->GetObject<Ipv4> ());
      routerStaticRouting->SetDefaultRoute (lanIpIfaces.at (r).GetAddress (0), 1);
    }

  // setting the wifi network usign wifiRouters as APs
  std::vector<NodeContainer> allWifiStations;
  std::vector<Ipv4InterfaceContainer> wifiRouterIpInterface;
  std::vector<Ipv4InterfaceContainer> wifiStationsIpInterface;
  std::vector<NetDeviceContainer> apNetDevices;

  //no RTS/CTS
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));
  //no fragmentation at MAC level
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("990000"));
  /* Setting maximum retransmissions limit */
  Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue (maxRetry));

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  ipv4h.SetBase ("192.168.0.0", "255.255.0.0", "0.0.0.3"); // resetting te base to comply with corresponding wifiRouters'
  uint32_t channelNumber = 1;
  for (uint32_t a = 0; a < ueNodes.GetN (); ++a)
    {
      NodeContainer wifiStations;
      wifiStations.Create (numberOfWifiNodes);

      YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
      channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
      channel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (wifiRange)); // 10th of distance

      phy.Set ("ChannelNumber", UintegerValue (channelNumber));
      phy.SetChannel (channel.Create ());

      switch (channelNumber) // alternating the channelNumber to obtain orthogonal adjacent frequencies
        {
          case 1:
            channelNumber = 6;
            break;
          case 6:
            channelNumber = 11;
            break;
          case 11:
            channelNumber = 1;
            break;
          default:
            continue;
        }

      WifiHelper wifi;
      wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);

      std::ostringstream oss;
      oss << "HtMcs" << mcs;
      wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (oss.str ()),
                               "ControlMode", StringValue (oss.str ()));

      std::ostringstream oss2;
      oss2 << "ns3-80211n-" << a;
      Ssid ssid = Ssid (oss2.str ());

      WifiMacHelper mac;
      mac.SetType ("ns3::ApWifiMac",
                  "Ssid", SsidValue (ssid));

      NetDeviceContainer apDevice;
      apDevice = wifi.Install (phy, mac, wifiRouters.Get (a));

      // Creating the bridge for the rest of the WLAN
      BridgeHelper bridge;
      NetDeviceContainer bridgeDev;
      bridgeDev = bridge.Install (wifiRouters.Get (a), NetDeviceContainer (apDevice, lanDevices.at (a).Get (1)));

      // assign AP IP address to bridge, not wifi
      Ipv4InterfaceContainer wifiRouterNodeInterface;
      wifiRouterNodeInterface = ipv4h.Assign (bridgeDev);

      // now the stations
      mac.SetType ("ns3::StaWifiMac",
                  "Ssid", SsidValue (ssid));

      NetDeviceContainer wifiStaDevice;
      wifiStaDevice = wifi.Install (phy, mac, wifiStations);

      // Set channel width
      Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));

      // Internet stack
      internet.Install (wifiStations);

      Ipv4InterfaceContainer wifiStaNodeInterface;
      wifiStaNodeInterface = ipv4h.Assign (wifiStaDevice);

      // Defining default routes for the wireless stations
      for (uint32_t r = 0; r < wifiStations.GetN (); ++r)
        {
          Ptr<Node> station = wifiStations.Get (r);
          Ptr<Ipv4StaticRouting> stationStaticRouting = ipv4RoutingHelper.GetStaticRouting (station->GetObject<Ipv4> ());
          stationStaticRouting->SetDefaultRoute (lanIpIfaces.at (a).GetAddress (0), 1);
        }


      // Mobility for wifi stations
      // stations will surround the AP using the RandomDiskPositionAllocator
      // with radius ap2NodesMaxDistance = distance/10.
      // Getting the Router's position to use as base for the stations'
      Ptr<MobilityModel> wifiRouterMm;
      Ptr<Node> wifiRouter = wifiRouters.Get (a);
      wifiRouterMm = wifiRouter->GetObject<MobilityModel> ();
      NS_ASSERT (wifiRouterMm != 0);
      Vector wifiRouterPos = wifiRouterMm->GetPosition ();

      double ap2NodesMaxDistance = wifiRange; // meters
      std::string theta = "ns3::UniformRandomVariable[Min=0|Max=6.2830]"; // the whole disk
      std::ostringstream oss3;
      oss3 << "ns3::UniformRandomVariable[Min=0.0" << "|Max=" << ap2NodesMaxDistance << "]";
      std::string rho = oss3.str();

      mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                      "X", DoubleValue (wifiRouterPos.x),
                                      "Y", DoubleValue (wifiRouterPos.y),
                                      "Theta", StringValue (theta),
                                      "Rho", StringValue (rho));

      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      mobility.Install (wifiStations);

      ipv4h.NewNetwork ();

      // saving all in containers
      allWifiStations.push_back (wifiStations);
      wifiRouterIpInterface.push_back (wifiRouterNodeInterface);
      wifiStationsIpInterface.push_back (wifiStaNodeInterface);
      apNetDevices.push_back (apDevice);
    }


  // Install and start applications on UEs and remote host
  uint16_t ulPort = 2000;
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;
  uint32_t payloadSize = 1472; //Bytes
  uint32_t txRate = 1; // Mbps
  double newInterval = (payloadSize * 8.0) / (txRate * 1e3); // msecs
  Ptr<FlowMonitor> monitor;
  FlowMonitorHelper flowHelper;

  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      ++ulPort;
      if (!wifiTraffic) // traffic does not traverse WiFi networks
        {
          if (remoteToWifi) // traffic originates at the remoteHost, towards a wifiRouter's Ethernet
            {
              NS_ASSERT (ueNodes.GetN () == 1);

              PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
              serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
              //serverApps.Add (dlPacketSinkHelper.Install (wifiRouters.Get (u)));

              //std::cout << "Receiving: " << wifiRouterIpInterface.at (u).GetAddress (0) << std::endl;
              //std::cout << "Receiving: " <<lanIpIfaces.at (u).GetAddress (0) << std::endl;

              UdpClientHelper dlClient (ueIpIface.GetAddress (u), ulPort);
              //UdpClientHelper dlClient (lanIpIfaces.at (u).GetAddress (0), ulPort);
              dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds(newInterval)));
              dlClient.SetAttribute ("MaxPackets", UintegerValue(4294967295u));
              dlClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));

              clientApps.Add (dlClient.Install (remoteHost));
            }
          else // traffic originates at the UE
            {
              PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));

              serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

              UdpClientHelper ulClient (remoteHostAddr, ulPort);
              ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(newInterval)));
              ulClient.SetAttribute ("MaxPackets", UintegerValue(4294967295u));
              ulClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));

              //clientApps.Add (ulClient.Install (ueNodes.Get(u)));
              clientApps.Add (ulClient.Install (wifiRouters.Get (u)));
            }
        }
      else // traffic originates at Wifi stations
        {
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
          for (uint16_t s = 0; s < allWifiStations.at (u).GetN (); ++s)
            {
              // Saving information from all wifiStations for the trace callbacks
              Ptr<Node> sta = allWifiStations.at (u).Get (s);
              Ptr<Ipv4> ip = sta->GetObject<Ipv4> ();
              Ipv4Address addr = ip->GetAddress (1,0).GetLocal ();
              tracer.ids[u][addr] = sta->GetId ();
              if (!genericMms)
                {
                  if (udpBasic)
                    {
                      std::cout <<"Simulation: Basic UDP generated at WiFi Stations" << std::endl;

                      UdpClientHelper ulClient (remoteHostAddr, ulPort);
                      ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(newInterval)));
                      ulClient.SetAttribute ("MaxPackets", UintegerValue(4294967295u));
                      ulClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));

                      clientApps.Add (ulClient.Install (allWifiStations.at (u).Get (s)));
                    }
                  else
                    { // to the report
                      payloadSize = 57; // loadsize of MMS response segment

                      std::ostringstream off_time;
                      off_time << "ns3::ConstantRandomVariable[Constant=" << OFF << "]";

                      std::cout <<"Simulation: On/Off=" << off_time.str() << ", UDP generated at WiFi Stations" << std::endl;

                      OnOffHelper ulClient ("ns3::UdpSocketFactory", (InetSocketAddress (remoteHostAddr, ulPort)));
                      ulClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));
                      ulClient.SetAttribute ("OnTime", StringValue ("ns3::UniformRandomVariable"));
                      ulClient.SetAttribute ("OffTime", StringValue (off_time.str()));
                      ulClient.SetAttribute ("DataRate", DataRateValue (DataRate (genericMmsDataRate)));

                      clientApps.Add (ulClient.Install (allWifiStations.at (u).Get (s)));
                    }
                }
              else
                {
                  payloadSize = 57; // loadsize of MMS response segment

                  PacketSinkHelper ulPacketSinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
                  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));

                  serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

                  // Install TCP/UDP Transmitter on the stations
                  OnOffHelper ulClient ("ns3::TcpSocketFactory", (InetSocketAddress (remoteHostAddr, ulPort)));
                  ulClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));

                  ulClient.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
                  //ulClient.SetAttribute ("OnTime", StringValue ("ns3::UniformRandomVariable"));
                  ulClient.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));

                  ulClient.SetAttribute ("DataRate", DataRateValue (DataRate (genericMmsDataRate)));
                  clientApps.Add (ulClient.Install (allWifiStations.at (u).Get (s)));
                }
            }

          if (enableFlowMon)
            {
              monitor = flowHelper.InstallAll ();
            }
          else
            {
              // PCAP tracing
              if (udpBasic)
                {
                  std::ostringstream trace;
                  trace << "generic-mms-wifi-trace-" << u;
                  phy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
                  phy.EnablePcap (trace.str (), apNetDevices.at (u));
                }
            }
        }
    }
  serverApps.Start (Seconds (0.5));

  /* Starting clientApps randomly within [0, OFF] */
  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  x->SetAttribute ("Min", DoubleValue (1.0));
  x->SetAttribute ("Max", DoubleValue (OFF));
  for (uint32_t  i = 0; i < clientApps.GetN (); i++)
    {
      clientApps.Get (i)->SetStartTime (Seconds (double (x->GetValue ())));
    }

  //lteHelper->EnableTraces (); // outputs .txt files
  // Uncomment to enable PCAP tracing
  //p2ph.EnablePcapAll("lena-epc-first");


  // Checking IPs
  std::cout << "LTE+EPC network specs" << std::endl;
  std::cout << "=====================" << std::endl,
  std::cout << "-RemoteHost: " << remoteHostAddr << std::endl;
  std::cout << "-pgw: " << internetIpIfaces.GetAddress (0) << std::endl;
  std::cout << "-eNodeB: " << epcHelper->GetUeDefaultGatewayAddress () << std::endl;

  for (uint32_t h = 0; h < ueIpIface.GetN (); ++h)
    {
      std::cout << "-UE-" << h << ": " << ueIpIface.GetAddress (h) << std::endl;
    }

  std::cout << std::endl;
  for (uint32_t i = 0; i < lanIpIfaces.size (); ++i)
    {
      std::cout << "-Network-" << i << std::endl;
      std::cout << "\t-UE Ethernet: " << lanIpIfaces.at (i).GetAddress (0) << std::endl;
      std::cout << "\t-Router Wifi-" << i << " Ethernet: " << lanIpIfaces.at (i).GetAddress (1) << std::endl << std::endl;;
    }

  std::cout << "WiFi network specs" << std::endl;
  std::cout << "===================" << std::endl;
  for (uint32_t j = 0; j < wifiRouters.GetN (); ++j)
    {
      Ptr<YansWifiPhy> phyRouter;
      phyRouter = apNetDevices.at (j).Get (0)->GetObject<WifiNetDevice> ()->GetPhy ()->GetObject<YansWifiPhy> ();

      std::cout << "-Router Wifi-" << j << " (Ch-" << phyRouter->GetChannelNumber () << ") : " <<
        wifiRouterIpInterface.at (j).GetAddress (0) << std::endl;
      for (uint32_t k = 0; k < wifiStationsIpInterface.at (j).GetN (); ++k)
        {
          std::cout << "\t-Station-" << k << ": " << wifiStationsIpInterface.at (j).GetAddress (k) << std::endl;
        }
      std::cout << std::endl;
    }

  // Checking positions
  std::cout << "Mobility details" << std::endl;
  std::cout << "================" << std::endl;
  for (uint32_t l = 0; l < ueNodes.GetN (); ++l)
    {
      std::cout << "-UE/AP-" << l << ": ";
      Vector pos = Vector (getNodePosition (ueNodes.Get (l)));
      std::cout << "X: " << pos.x << ", Y: " << pos.y << ", Z: " << pos.z << std::endl;

      for  (uint32_t m = 0; m < allWifiStations.at (l).GetN (); ++m)
        {
          std::cout << "\t-Station-" << m << ": ";
          Vector pos2 = Vector (getNodePosition (allWifiStations.at (l).Get (m)));
          std::cout << "X: " << pos2.x << ", Y: " << pos2.y << ", Z: " << pos2.z << std::endl;

        }
     }



  // Connecting trace sources
  Config::ConnectWithoutContext ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
                  MakeBoundCallback(&TraceRxPackets, &results, &tracer));

  Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",
                  MakeBoundCallback(&TraceGenPackets, &results, &tracer));


  simTime += double (1.0 + OFF); // not all sources start at the same time
  Simulator::Stop (Seconds (simTime));
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

  if (!enableFlowMon) // no flowMonitor and conventional UDP traffic
    {
      if (!genericMms)
        {
          for (uint32_t a = 0; a < serverApps.GetN (); ++a)
            {
              sink = StaticCast<PacketSink> (serverApps.Get (a));
              throughput = ((sink->GetTotalRx () * 8) / (1e6  * simTime));
              avg_delay = (std::accumulate (results.delay[a].begin (), results.delay[a].end (), 0.0))
                / results.delay[a].size () * 1.0;
              pdr = results.numRxPackets[a] / results.numGenPackets[a];

              std::cout << "Network-" << a << ")--->Throughput : " << throughput << " Mbps, ";
              std::cout << "Delay: " << avg_delay << " us, ";
              std::cout << "PDR: " << pdr << "/1" << std::endl;

              avgThroughput.push_back (throughput);
              overallDelay.push_back (avg_delay);
              averagePdr.push_back (pdr);

              // writing per AP metrics
              writePerApMetrics (a, throughput, avg_delay, pdr);
            }

          double relativeService = results.rxBytes / results.genBytes;
          double generationRate = results.genBytes * 8 / (1e6 * simTime);
          std::cout << std::endl << "--->Overall: Throughput (" << results.rxBytes * 8 / (1e6 * simTime) <<
            " Mbps) / Generation rate (" << generationRate << " Mbps) : " <<
            relativeService << "/1" << std::endl;

          writeOverallResultsToFile (serverApps.GetN (), avgThroughput, overallDelay, averagePdr, generationRate, filename, numberOfWifiNodes);
        }
      else
        {
          // no FlowMonitor and genericMms (PCAP?)
        }
    }
  else
    {
      monitor->CheckForLostPackets ();
      Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
      FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

      for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin ();
        i != stats.end (); ++i)
        {
          Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);

          std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
          std::cout << "  -Tx Packets: " << i->second.txPackets << "\n";
          std::cout << "  -Rx packets: " << i->second.rxPackets << "\n";
          std::cout << "  -Delay sum: " << i->second.delaySum.GetMicroSeconds () * 1.0 << " us\n";
          std::cout << "  -Tx Bytes:   " << (i->second.txBytes)  << "\n";
          std::cout << "  -TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000  << " Mbps\n";
          std::cout << "  -Delay:      "  << i->second.lastDelay.GetMicroSeconds () << " us\n";
        }


    }

  Simulator::Destroy();
  return 0;

}
