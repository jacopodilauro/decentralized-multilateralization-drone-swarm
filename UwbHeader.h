#ifndef UWB_HEADER_H
#define UWB_HEADER_H

#include "ns3/header.h"
#include <iostream>

namespace ns3 {

    class UwbHeader : public Header 
    {
    public:
        static TypeId GetTypeId (void);
        virtual TypeId GetInstanceTypeId (void) const override;
        
        UwbHeader ();
        virtual ~UwbHeader ();

        virtual uint32_t GetSerializedSize (void) const override;
        virtual void Serialize (Buffer::Iterator start) const override;
        virtual uint32_t Deserialize (Buffer::Iterator start) override;
        virtual void Print (std::ostream &os) const override;

        void SetSenderId (uint32_t id);
        void SetTxTimestampPs (uint64_t timestamp_ps);
        void SetGpsPosition (double x, double y, double z);
        //void SetVoteBitmask (uint32_t mask);
        void SetImLeaving (bool leaving);

        void SetSharedRange(uint32_t targetId, double range);
        double GetSharedRange(uint32_t targetId) const;

        uint32_t GetSenderId () const;
        uint64_t GetTxTimestampPs () const;
        double GetGpsX () const;
        double GetGpsY () const;
        double GetGpsZ () const;
        //uint32_t GetVoteBitmask () const;
        bool GetImLeaving () const;
        void SetAlarmsList(const std::vector<uint8_t>& alarms);
        std::vector<uint8_t> GetAlarmsList() const;

        void SetGossipLeaves(const std::vector<uint32_t>& leaves);
        std::vector<uint32_t> GetGossipLeaves() const;

        void SetGossipEvictions(std::vector<uint32_t> evictions);
        std::vector<uint32_t> GetGossipEvictions() const;

    private:
        uint32_t m_senderId;
        uint64_t m_txTimestampPs;
        double m_gpsX;
        double m_gpsY;
        double m_gpsZ;
        uint32_t m_voteBitmask;
        bool m_imLeaving;

        std::vector<double> m_sharedRanges;
        std::vector<uint8_t> m_alarmsList;
        std::vector<uint32_t> m_gossipLeaves;
        std::vector<uint32_t> m_gossipEvictions;
    };
} 
#endif