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

UwbHeader::UwbHeader () : m_senderId(0), m_txTimestampPs(0), m_gpsX(0), m_gpsY(0), m_gpsZ(0), m_voteBitmask(0xFFFFFFFF) {
    //for(int i=0; i<6; i++) m_sharedRanges[i] = -1.0;
    m_sharedRanges.clear();
}
UwbHeader::~UwbHeader () {}

uint32_t UwbHeader::GetSerializedSize (void) const {
    // 4(id) + 8(time) + 24(gps) + 4(bitmask) + 4(n_ranges) + (n_ranges * 8)
    return 4 + 8 + 24 + 4 + 4 + (m_sharedRanges.size() * 8); 
}

void UwbHeader::Serialize (Buffer::Iterator start) const {
    start.WriteHtonU32 (m_senderId);
    start.WriteHtonU64 (m_txTimestampPs);

    uint64_t x_bytes, y_bytes, z_bytes;
    std::memcpy(&x_bytes, &m_gpsX, 8); std::memcpy(&y_bytes, &m_gpsY, 8); std::memcpy(&z_bytes, &m_gpsZ, 8);
    start.WriteU64 (x_bytes); start.WriteU64 (y_bytes); start.WriteU64 (z_bytes);
    
    start.WriteHtonU32 (m_voteBitmask);
    
    start.WriteHtonU32 (m_sharedRanges.size());
    for(double r : m_sharedRanges) {
        uint64_t range_bytes;
        std::memcpy(&range_bytes, &r, 8);
        start.WriteU64(range_bytes);
    }
}

uint32_t UwbHeader::Deserialize (Buffer::Iterator start) {
    m_senderId = start.ReadNtohU32 ();
    m_txTimestampPs = start.ReadNtohU64 ();
    
    uint64_t x_bytes = start.ReadU64 (), y_bytes = start.ReadU64 (), z_bytes = start.ReadU64 ();
    std::memcpy(&m_gpsX, &x_bytes, 8); std::memcpy(&m_gpsY, &y_bytes, 8); std::memcpy(&m_gpsZ, &z_bytes, 8);
    m_voteBitmask = start.ReadNtohU32 ();
    
    uint32_t n_ranges = start.ReadNtohU32();
    m_sharedRanges.resize(n_ranges);
    
    for(uint32_t i = 0; i < n_ranges; i++) {
        uint64_t range_bytes = start.ReadU64();
        std::memcpy(&m_sharedRanges[i], &range_bytes, 8);
    }
    return GetSerializedSize (); 
}

void UwbHeader::Print (std::ostream &os) const {}
void UwbHeader::SetSenderId (uint32_t id) { m_senderId = id; }
void UwbHeader::SetTxTimestampPs (uint64_t timestamp_ps) { m_txTimestampPs = timestamp_ps; }
void UwbHeader::SetGpsPosition (double x, double y, double z) { m_gpsX = x; m_gpsY = y; m_gpsZ = z; }
void UwbHeader::SetVoteBitmask (uint32_t mask) { m_voteBitmask = mask; }
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
uint32_t UwbHeader::GetVoteBitmask () const { return m_voteBitmask; }
double UwbHeader::GetSharedRange(uint32_t targetId) const 
{ 
    if (targetId < m_sharedRanges.size()) {
        return m_sharedRanges[targetId];
    }
    return -1.0;    
}

} 
