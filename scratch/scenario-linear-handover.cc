/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Linear Handover Scenario - Fast deterministic handover testing
 *
 * Topology: 1 LTE eNB (anchor) + 2 mmWave gNBs, 1 UE (Dual Connectivity)
 * Movement: Linear path from gNB1 to gNB2 at 80 m/s
 * Purpose: Phase 2 HTTP integration testing - validates xApp handover detection
 *
 * The LTE eNB acts as control plane anchor, while mmWave gNBs provide
 * high-throughput data connectivity. Secondary cell handovers between
 * mmWave cells are managed by the LteEnbRrc via X2 interface.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include <ns3/lte-ue-net-device.h>
#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "../src/mmwave/model/node-container-manager.h"
#include "ns3/lte-helper.h"
#include "ns3/isotropic-antenna-model.h"

using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE ("ScenarioLinearHandover");

// Global configuration values
static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (10),
                                      ns3::MakeUintegerChecker<uint32_t> ());

static ns3::GlobalValue g_enableTraces ("enableTraces", "If true, generate ns-3 traces",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2lteEnabled ("e2lteEnabled", "If true, send LTE E2 reports",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2nrEnabled ("e2nrEnabled", "If true, send NR E2 reports",
                                       ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2du ("e2du", "If true, send DU reports", ns3::BooleanValue (true),
                                ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuUp ("e2cuUp", "If true, send CU-UP reports", ns3::BooleanValue (true),
                                  ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuCp ("e2cuCp", "If true, send CU-CP reports", ns3::BooleanValue (true),
                                  ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_reducedPmValues ("reducedPmValues", "If true, use a subset of the pm containers",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue
    g_hoSinrDifference ("hoSinrDifference",
                        "The value for which a handover between MmWave eNB is triggered",
                        ns3::DoubleValue (2), ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue
    g_indicationPeriodicity ("indicationPeriodicity",
                             "E2 Indication Periodicity reports (value in seconds)",
                             ns3::DoubleValue (0.1), ns3::MakeDoubleChecker<double> (0.01, 2.0));

static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (30),
                                   ns3::MakeDoubleChecker<double> (10.0, 1000.0));

static ns3::GlobalValue g_outageThreshold ("outageThreshold",
                                           "SNR threshold for outage events [dB]",
                                           ns3::DoubleValue (-5.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_numberOfRaPreambles (
    "numberOfRaPreambles",
    "how many random access preambles are available for the contention based RACH process",
    ns3::UintegerValue (40),
    ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue
    g_handoverMode ("handoverMode",
                    "HO euristic to be used, can be only \"NoAuto\", \"FixedTtt\", \"DynamicTtt\", \"Threshold\"",
                    ns3::StringValue ("DynamicTtt"), ns3::MakeStringChecker ());

static ns3::GlobalValue g_e2TermIp ("e2TermIp", "The IP address of the RIC E2 termination",
                                    ns3::StringValue ("127.0.0.1"), ns3::MakeStringChecker ());

static ns3::GlobalValue
    g_enableE2FileLogging ("enableE2FileLogging",
                           "If true, generate offline file logging instead of connecting to RIC",
                           ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_controlFileName ("controlFileName",
                                           "The path to the control file (can be absolute)",
                                           ns3::StringValue (""),
                                           ns3::MakeStringChecker ());

static ns3::GlobalValue g_e2_func_id("KPM_E2functionID", "KPM Function ID to subscribe",
                                     ns3::DoubleValue(2), ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_rc_e2_func_id("RC_E2functionID", "RC Function ID to subscribe",
                                        ns3::DoubleValue(3), ns3::MakeDoubleChecker<double>());

int
main (int argc, char *argv[])
{
  // Enable logging for KPM indications and handover tracking
  LogComponentEnable ("KpmIndication", LOG_LEVEL_DEBUG);
  LogComponentEnable ("MmWaveHelper", LOG_LEVEL_INFO);
  LogComponentEnable ("McStatsCalculator", LOG_LEVEL_INFO);

  // Scenario dimensions
  double maxXAxis = 2000;
  double maxYAxis = 2000;

  // Command line arguments
  CommandLine cmd;
  cmd.Parse (argc, argv);

  bool harqEnabled = true;

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;

  GlobalValue::GetValueByName ("hoSinrDifference", doubleValue);
  double hoSinrDifference = doubleValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("enableTraces", booleanValue);
  bool enableTraces = booleanValue.Get ();
  GlobalValue::GetValueByName ("handoverMode", stringValue);
  std::string handoverMode = stringValue.Get ();
  GlobalValue::GetValueByName ("outageThreshold", doubleValue);
  double outageThreshold = doubleValue.Get ();
  GlobalValue::GetValueByName ("numberOfRaPreambles", uintegerValue);
  uint8_t numberOfRaPreambles = static_cast<uint8_t> (uintegerValue.Get ());
  GlobalValue::GetValueByName ("e2TermIp", stringValue);
  std::string e2TermIp = stringValue.Get ();
  GlobalValue::GetValueByName ("enableE2FileLogging", booleanValue);
  bool enableE2FileLogging = booleanValue.Get ();
  GlobalValue::GetValueByName ("controlFileName", stringValue);
  std::string controlFilename = stringValue.Get ();
  GlobalValue::GetValueByName ("indicationPeriodicity", doubleValue);
  double indicationPeriodicity = doubleValue.Get ();
  GlobalValue::GetValueByName ("e2du", booleanValue);
  bool e2du = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2cuUp", booleanValue);
  bool e2cuUp = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2cuCp", booleanValue);
  bool e2cuCp = booleanValue.Get ();
  GlobalValue::GetValueByName ("reducedPmValues", booleanValue);
  bool reducedPmValues = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2lteEnabled", booleanValue);
  bool e2lteEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2nrEnabled", booleanValue);
  bool e2nrEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("KPM_E2functionID", doubleValue);
  uint16_t kpm_e2_func_id = (uint16_t) doubleValue.Get ();
  GlobalValue::GetValueByName ("RC_E2functionID", doubleValue);
  uint16_t rc_e2_func_id = (uint16_t) doubleValue.Get ();

  NS_LOG_UNCOND ("Linear Handover Scenario");
  NS_LOG_UNCOND ("=======================");
  NS_LOG_UNCOND ("Topology: 2 mmWave gNBs, 1 UE");
  NS_LOG_UNCOND ("Movement: Linear path at 80 m/s");
  NS_LOG_UNCOND ("Expected handover: ~2-3 seconds");
  NS_LOG_UNCOND ("Handover mode: " << handoverMode);
  NS_LOG_UNCOND ("HO SINR difference: " << hoSinrDifference << " dB");
  NS_LOG_UNCOND ("E2 Term IP: " << e2TermIp);
  NS_LOG_UNCOND ("Indication periodicity: " << indicationPeriodicity << " s");

  // RLC buffer configuration
  Config::SetDefault ("ns3::LteRlcUm::ReportBufferStatusTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                      TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                      UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));

  // Handover configuration
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (outageThreshold));
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", StringValue (handoverMode));
  Config::SetDefault ("ns3::LteEnbRrc::HoSinrDifference", DoubleValue (hoSinrDifference));

  // mmWave configuration
  double bandwidth = 20e6;  // 20 MHz
  double centerFrequency = 28e9;  // 28 GHz mmWave
  int numAntennasMcUe = 1;
  int numAntennasMmWave = 1;

  Config::SetDefault ("ns3::McUeNetDevice::AntennaNum", UintegerValue (numAntennasMcUe));
  Config::SetDefault ("ns3::MmWaveNetDevice::AntennaNum", UintegerValue (numAntennasMmWave));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));

  // E2 Function ID configuration
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::KPM_E2functionID", DoubleValue (kpm_e2_func_id));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::RC_E2functionID", DoubleValue (rc_e2_func_id));

  // E2 Periodicity - Indication reporting period
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));

  // E2 Termination IP address (FlexRIC RIC)
  Config::SetDefault ("ns3::MmWaveHelper::E2TermIp", StringValue (e2TermIp));

  // E2 Mode Configuration (use command-line variables)
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeLte", BooleanValue (e2lteEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeNr", BooleanValue (e2nrEnabled));

  // Enable E2 reports (use command-line variables)
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue (e2du));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue (e2cuUp));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue (e2cuCp));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));

  // Enable/disable E2 file logging
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableE2FileLogging", BooleanValue (enableE2FileLogging));

  // LTE eNB E2 Configuration (for DC anchor - control plane only)
  Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue (controlFilename));
  Config::SetDefault ("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue (e2cuUp));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue (e2cuCp));
  Config::SetDefault ("ns3::LteEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableE2FileLogging", BooleanValue (enableE2FileLogging));

  // DEBUG: Verify E2 configuration
  NS_LOG_INFO ("[E2 DEBUG] E2 Configuration Applied:");
  NS_LOG_INFO ("  E2TermIp: " << e2TermIp);
  NS_LOG_INFO ("  KPM Function ID: " << kpm_e2_func_id);
  NS_LOG_INFO ("  RC Function ID: " << rc_e2_func_id);
  NS_LOG_INFO ("  E2 Periodicity: " << indicationPeriodicity);
  NS_LOG_INFO ("  E2ModeLte: " << (e2lteEnabled ? "ENABLED" : "DISABLED"));
  NS_LOG_INFO ("  E2ModeNr: " << (e2nrEnabled ? "ENABLED" : "DISABLED"));
  NS_LOG_INFO ("  EnableDuReport: " << (e2du ? "ENABLED" : "DISABLED"));
  NS_LOG_INFO ("  EnableCuUpReport: " << (e2cuUp ? "ENABLED" : "DISABLED"));
  NS_LOG_INFO ("  EnableCuCpReport: " << (e2cuCp ? "ENABLED" : "DISABLED"));
  NS_LOG_INFO ("  ReducedPmValues: " << (reducedPmValues ? "ENABLED" : "DISABLED"));
  NS_LOG_INFO ("  EnableE2FileLogging: " << (enableE2FileLogging ? "ENABLED" : "DISABLED"));

  // Create helpers
  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);

  // Topology: 1 LTE eNB (anchor) + 2 mmWave gNBs, 1 UE (Dual Connectivity)
  uint8_t nLteEnbNodes = 1;
  uint8_t nMmWaveEnbNodes = 2;
  uint8_t nUeNodes = 1;

  // Get SGW/PGW and create remote host
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create Internet connection
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Create nodes (LTE anchor + mmWave gNBs + UEs)
  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  lteEnbNodes.Create (nLteEnbNodes);
  mmWaveEnbNodes.Create (nMmWaveEnbNodes);
  ueNodes.Create (nUeNodes);
  allEnbNodes.Add (lteEnbNodes);
  allEnbNodes.Add (mmWaveEnbNodes);
  NodeContainerManager::GetInstance().SetMmWaveEnbNodes(mmWaveEnbNodes);

  // Position eNBs/gNBs:
  // LTE eNB at center (650, 1000) - acts as control plane anchor
  // mmWave gNB1 at (500, 1000), mmWave gNB2 at (800, 1000)
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (650, 1000, 25));  // LTE eNB (center, higher tower)
  enbPositionAlloc->Add (Vector (500, 1000, 10));  // mmWave gNB 1
  enbPositionAlloc->Add (Vector (800, 1000, 10));  // mmWave gNB 2

  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);  // Install on all eNBs (LTE + mmWave)

  // UE starts near gNB1 and moves in straight line toward gNB2
  // Start position: (400, 1000) - 100m from gNB1
  // Velocity: 80 m/s toward gNB2 (horizontal movement) - FAST!
  // Distance to handover zone: ~150m
  // Expected handover: around 2-3 seconds when UE is between the two gNBs

  MobilityHelper uemobility;
  Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
  uePositionAlloc->Add (Vector (400, 1000, 1.5));  // Start closer to gNB1

  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uemobility.Install (ueNodes);

  // Set UE velocity: 80 m/s horizontally toward gNB2 - FAST!
  Ptr<ConstantVelocityMobilityModel> ueModel = ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
  ueModel->SetVelocity(Vector(80.0, 0.0, 0.0));  // 80 m/s in +X direction

  NS_LOG_UNCOND ("LTE eNB position: (650, 1000, 25) - control plane anchor");
  NS_LOG_UNCOND ("mmWave gNB1 position: (500, 1000, 10)");
  NS_LOG_UNCOND ("mmWave gNB2 position: (800, 1000, 10)");
  NS_LOG_UNCOND ("UE start position: (400, 1000, 1.5)");
  NS_LOG_UNCOND ("UE velocity: 80 m/s toward gNB2");

  // Install LTE, mmWave, and MC UE devices (Dual Connectivity)
  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
  NetDeviceContainer mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);

  // Install IP stack on UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));

  // Set default gateway for UE
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Add X2 interfaces between LTE and mmWave eNBs (required for DC handovers)
  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);

  // Attach MC UE to closest eNBs (LTE anchor + mmWave for data)
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs, lteEnbDevs);

  // Install applications - UDP downlink traffic
  uint16_t portUdp = 1234;
  ApplicationContainer sinkApp;
  ApplicationContainer clientApp;

  // Packet sink on UE
  PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                       InetSocketAddress (Ipv4Address::GetAny (), portUdp));
  sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (0)));

  // UDP client on remote host sending to UE
  UdpClientHelper dlClient (ueIpIface.GetAddress (0), portUdp);
  dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (500)));
  dlClient.SetAttribute ("MaxPackets", UintegerValue (UINT32_MAX));
  dlClient.SetAttribute ("PacketSize", UintegerValue (1280));
  clientApp.Add (dlClient.Install (remoteHost));

  // Start applications
  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();
  sinkApp.Start (Seconds (0));
  clientApp.Start (MilliSeconds (100));
  clientApp.Stop (Seconds (simTime - 0.1));

  // Enable traces
  if (enableTraces)
    {
      mmwaveHelper->EnableTraces ();
    }

  NS_LOG_UNCOND ("Simulation time: " << simTime << " seconds");
  NS_LOG_UNCOND ("Starting simulation...");
  NS_LOG_INFO ("[E2 DEBUG] Simulation starting - E2 interface should initialize now");
  NS_LOG_INFO ("[E2 DEBUG] Expecting E2 connection to " << e2TermIp << " on SCTP");

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  NS_LOG_UNCOND ("Simulation complete.");
  return 0;
}
