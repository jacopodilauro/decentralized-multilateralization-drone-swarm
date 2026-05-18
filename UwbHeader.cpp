#include "UwbHeader.h"
#include "ns3/log.h"
#include <cstring>
 
namespace ns3 {
 
NS_LOG_COMPONENT_DEFINE ("UwbHeader");
NS_OBJECT_ENSURE_REGISTERED (UwbHeader);

TypeId UwbHeader::GetTypeId (void) {
    static TypeId tid = TypeId ("ns3::UwbHeader").SetParent<Header> ().SetGroupName("Custom").AddConstructor<UwbHeader> ();
    return tid;
}
TypeId UwbHeader::GetInstanceTypeId (void) const { return GetTypeId (); }

UwbHeader::UwbHeader () : m_senderId(0), m_txTimestampPs(0), m_gpsX(0), m_gpsY(0), m_gpsZ(0), m_imLeaving(false) {
    m_sharedRanges.clear();
    m_gossipLeaves.clear();
    m_alarmsList.clear(); // Aggiungiamo questo
}
UwbHeader::~UwbHeader () {}

uint32_t UwbHeader::GetSerializedSize(void) const {
    uint32_t size = 0;   
    // 37 byte in totale
    size += 4;  // m_senderId
    size += 8;  // m_txTimestampPs
    size += 24; // m_gpsX, m_gpsY, m_gpsZ
    size += 1;  // m_imLeaving  
    // (Ultimi usciti)
    size += 1;                           // 1 byte per dire al ricevitore QUANTI elementi ci sono
    size += (m_gossipLeaves.size() * 4); // 4 byte (uint32_t) per ogni ID drone   
    // 3. Il vettore "Allarmi" dinamico
    size += 1;                           // 1 byte per dire QUANTI allarmi ci sono (max 255 droni)
    size += (m_alarmsList.size() * 1);   // 1 byte (uint8_t) per ogni allarme (La tua intuizione!)    
    // 4. Il vettore "Ranges" condivisi
    size += 4;                           // 4 byte per dire QUANTE distanze ci sono
    size += (m_sharedRanges.size() * 4); // 8 byte per ogni double (i double pesano 8 byte, non 4!)

    return size;
}

void UwbHeader::Serialize (Buffer::Iterator start) const {
    start.WriteHtonU32 (m_senderId);
    start.WriteHtonU64 (m_txTimestampPs);

    uint64_t x_bytes, y_bytes, z_bytes;
    std::memcpy(&x_bytes, &m_gpsX, 8); std::memcpy(&y_bytes, &m_gpsY, 8); std::memcpy(&z_bytes, &m_gpsZ, 8);
    start.WriteU64 (x_bytes); start.WriteU64 (y_bytes); start.WriteU64 (z_bytes);
    
    start.WriteU8 (m_imLeaving ? 1 : 0);
    
    // Scriviamo Gossip
    start.WriteU8(m_gossipLeaves.size());
    for(uint32_t id : m_gossipLeaves) {
        start.WriteHtonU32(id);
    }

    // Scriviamo Allarmi
    start.WriteU8(m_alarmsList.size());
    for(uint8_t alarm : m_alarmsList) {
        start.WriteU8(alarm);
    }
    
    // Scriviamo Ranges
    start.WriteHtonU32 (m_sharedRanges.size());
    for(double r : m_sharedRanges) {
        if (r < 0.0) {
            start.WriteHtonU32(0xFFFFFFFF);
        } else {
            uint32_t range_mm = static_cast<uint32_t>(r * 1000.0);
            start.WriteHtonU32(range_mm);
        }
    }
}

uint32_t UwbHeader::Deserialize (Buffer::Iterator start) {
    m_senderId = start.ReadNtohU32 ();
    m_txTimestampPs = start.ReadNtohU64 ();
    
    uint64_t x_bytes = start.ReadU64 (), y_bytes = start.ReadU64 (), z_bytes = start.ReadU64 ();
    std::memcpy(&m_gpsX, &x_bytes, 8); std::memcpy(&m_gpsY, &y_bytes, 8); std::memcpy(&m_gpsZ, &z_bytes, 8);
    
    m_imLeaving = (start.ReadU8 () != 0);
    
    // Leggiamo Gossip
    uint8_t gossipSize = start.ReadU8();
    m_gossipLeaves.clear();
    for(int i = 0; i < gossipSize; i++) {
        m_gossipLeaves.push_back(start.ReadNtohU32());
    }

    // Leggiamo Allarmi
    uint8_t alarmsSize = start.ReadU8();
    m_alarmsList.clear();
    for(int i = 0; i < alarmsSize; i++) {
        m_alarmsList.push_back(start.ReadU8());
    }
    
    // Leggiamo Ranges
    uint32_t n_ranges = start.ReadNtohU32();
    m_sharedRanges.resize(n_ranges);
    for(uint32_t i = 0; i < n_ranges; i++) {
        uint32_t range_mm = start.ReadNtohU32();
        if (range_mm == 0xFFFFFFFF) {
            m_sharedRanges[i] = -1.0;
        } else {
            m_sharedRanges[i] = static_cast<double>(range_mm) / 1000.0;
        }
    }
    return GetSerializedSize (); 
}

void UwbHeader::Print (std::ostream &os) const {}
void UwbHeader::SetSenderId (uint32_t id) { m_senderId = id; }
void UwbHeader::SetTxTimestampPs (uint64_t timestamp_ps) { m_txTimestampPs = timestamp_ps; }
void UwbHeader::SetGpsPosition (double x, double y, double z) { m_gpsX = x; m_gpsY = y; m_gpsZ = z; }
//void UwbHeader::SetVoteBitmask (uint32_t mask) { m_voteBitmask = mask; }
void UwbHeader::SetImLeaving (bool leaving) { m_imLeaving = leaving; }
void UwbHeader::SetSharedRange(uint32_t targetId, double range) 
{
    if (targetId >= m_sharedRanges.size()) {
        m_sharedRanges.resize(targetId + 1, -1.0);
    }
    m_sharedRanges[targetId] = range; 
}

uint32_t UwbHeader::GetSenderId () const { return m_senderId; }
uint64_t UwbHeader::GetTxTimestampPs () const { return m_txTimestampPs; }
double UwbHeader::GetGpsX () const { return m_gpsX; }
double UwbHeader::GetGpsY () const { return m_gpsY; }
double UwbHeader::GetGpsZ () const { return m_gpsZ; }
//uint32_t UwbHeader::GetVoteBitmask () const { return m_voteBitmask; }
bool     UwbHeader::GetImLeaving ()   const { return m_imLeaving; }
double UwbHeader::GetSharedRange(uint32_t targetId) const 
{ 
    if (targetId < m_sharedRanges.size()) {
        return m_sharedRanges[targetId];
    }
    return -1.0;    
}

void UwbHeader::SetAlarmsList(const std::vector<uint8_t>& alarms) { m_alarmsList = alarms; }
std::vector<uint8_t> UwbHeader::GetAlarmsList() const { return m_alarmsList; }

void UwbHeader::SetGossipLeaves(const std::vector<uint32_t>& leaves) { m_gossipLeaves = leaves; }
std::vector<uint32_t> UwbHeader::GetGossipLeaves() const { return m_gossipLeaves; }
}