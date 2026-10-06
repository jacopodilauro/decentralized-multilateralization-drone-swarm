#ifndef UWB_HEADER_H
#define UWB_HEADER_H

#include "ns3/header.h"
#include <map>
#include <cmath>
#include <algorithm>
#include <vector>
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
        void SetImLeaving (bool leaving);

        // age = secondi trascorsi tra la misura del range e la trasmissione del pacchetto
        void SetSharedRange(uint32_t targetId, double range, double age = 0.0);
        double GetSharedRange(uint32_t targetId) const;
        double GetSharedRangeAge(uint32_t targetId) const;   // -1 se il range non c'e'

        // --- DS-TWR (presenti nel pacchetto solo se SetDsTwrMode(true)) ---
        // Ricezione riportata: ho ricevuto il pacchetto 'seq' del drone 'id' all'istante 'rxStamp' (mio orologio)
        // los: la ricezione era in vista diretta (diagnostica del primo cammino del chip UWB)
        struct RxReport { uint32_t id; uint8_t seq; uint64_t rxStamp; bool los = true; };
        static void SetDsTwrMode(bool on) { s_dsTwr = on; }
        // Accuratezza dichiarata dal GPS (orizz./vert., in cm): presente solo se SetGnssAccMode(true)
        static void SetGnssAccMode(bool on) { s_gnssAcc = on; }
        void SetGpsAcc(double hAccM, double vAccM) { m_hAccCm = ToCm(hAccM); m_vAccCm = ToCm(vAccM); }
        double GetGpsHAcc() const { return m_hAccCm / 100.0; }
        double GetGpsVAcc() const { return m_vAccCm / 100.0; }
        static uint16_t ToCm(double m) { double c = std::round(m * 100.0); return (uint16_t)std::min(65535.0, std::max(1.0, c)); }
        static bool DsTwrMode() { return s_dsTwr; }
        void SetUwbTx(uint8_t seq, uint64_t txStamp) { m_seq = seq; m_uwbTxStamp = txStamp; }
        uint8_t  GetSeq() const { return m_seq; }
        uint64_t GetUwbTxStamp() const { return m_uwbTxStamp; }
        void SetRxReports(const std::vector<RxReport>& r) { m_rxReports = r; }
        const std::vector<RxReport>& GetRxReports() const { return m_rxReports; }

        uint32_t GetSenderId () const;
        uint64_t GetTxTimestampPs () const;
        double GetGpsX () const;
        double GetGpsY () const;
        double GetGpsZ () const;
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
        bool m_imLeaving;

        // Range condivisi: solo quelli effettivamente misurati (ID bersaglio -> range [m], eta' [s]).
        // Sul pacchetto: 1 byte = numero di range, poi per ognuno 2 byte ID + 4 byte range in mm
        // + 2 byte eta' in decimi di millisecondo (max 6.5535 s).
        std::map<uint32_t, std::pair<double, double>> m_sharedRanges;

        // DS-TWR: 1 byte seq + 5 byte timestamp di trasmissione + 1 byte numero di ricezioni
        // + per ciascuna 2 byte ID + 1 byte (7 bit seq + 1 bit LOS) + 5 byte timestamp di ricezione
        static inline bool s_dsTwr = false;
        static inline bool s_gnssAcc = false;
        uint16_t m_hAccCm = 0, m_vAccCm = 0;
        uint8_t  m_seq = 0;
        uint64_t m_uwbTxStamp = 0;
        std::vector<RxReport> m_rxReports;
        std::vector<uint8_t> m_alarmsList;
        std::vector<uint32_t> m_gossipLeaves;
        std::vector<uint32_t> m_gossipEvictions;
    };
} 
#endif