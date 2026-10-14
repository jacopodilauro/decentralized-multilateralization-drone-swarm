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
        // Il mittente ritiene lo sciame catturato da uno spoofer esterno (Fase 8): bit 1 del byte di imLeaving
        void SetSwarmCapture(bool c) { m_swarmCapture = c; }
        // Velocita' misurata dai sensori di bordo (8.2c), solo sotto cattura: bit 2 dei flag + 3 x int16 in cm/s
        void SetImuVelocity(double vx, double vy, double vz) { m_hasImuVel = true; m_imuVel[0] = vx; m_imuVel[1] = vy; m_imuVel[2] = vz; }
        bool HasImuVelocity() const { return m_hasImuVel; }
        double GetImuVelocity(int i) const { return m_imuVel[i]; }
        // PPS marcato con l'orologio UWB del mittente (9.1), solo sotto cattura: bit 3 dei flag + secondo (mod 256,
        // 1 byte) + timestamp a 40 bit (5 byte)
        void SetPps(uint8_t sec, uint64_t stamp) { m_hasPps = true; m_ppsSec = sec; m_ppsStamp = stamp & ((1ULL << 40) - 1); }
        bool HasPps() const { return m_hasPps; }
        uint8_t GetPpsSec() const { return m_ppsSec; }
        uint64_t GetPpsStamp() const { return m_ppsStamp; }
        bool GetSwarmCapture() const { return m_swarmCapture; }

        // age = secondi trascorsi tra la misura del range e la trasmissione del pacchetto
        // los: la misura era in vista diretta (bit alto del campo range, nessun byte in piu')
        void SetSharedRange(uint32_t targetId, double range, double age = 0.0, bool los = true);
        double GetSharedRange(uint32_t targetId) const;
        double GetSharedRangeAge(uint32_t targetId) const;   // -1 se il range non c'e'
        bool   GetSharedRangeLos(uint32_t targetId) const;   // true se il range non c'e'

        // --- DS-TWR (presenti nel pacchetto solo se SetDsTwrMode(true)) ---
        // Ricezione riportata: ho ricevuto il pacchetto 'seq' del drone 'id' all'istante 'rxStamp' (mio orologio)
        // los: la ricezione era in vista diretta (diagnostica del primo cammino del chip UWB)
        struct RxReport { uint32_t id; uint8_t seq; uint64_t rxStamp; bool los = true; };
        static void SetDsTwrMode(bool on) { s_dsTwr = on; }
        // Covarianza dell'errore GPS individuale dichiarata dal ricevitore (presente solo se SetGnssAccMode(true)):
        // deviazioni standard E, N, U in cm (uint16) e correlazioni EN, EU, NU in 1/127 (int8): 9 byte.
        // cov = {cEE, cNN, cUU, cEN, cEU, cNU} [m^2]
        static void SetGnssAccMode(bool on) { s_gnssAcc = on; }
        void SetGpsCov(const double cov[6]) {
            double sd[3];
            for (int i = 0; i < 3; i++) { sd[i] = std::sqrt(std::max(0.0, cov[i])); m_sdCm[i] = ToCm(sd[i]); }
            const int a[3] = { 0, 0, 1 }, b[3] = { 1, 2, 2 };
            for (int k = 0; k < 3; k++) {
                double r = cov[3 + k] / std::max(1e-12, sd[a[k]] * sd[b[k]]);
                m_rho[k] = (int8_t)std::lround(std::clamp(r, -1.0, 1.0) * 127.0);
            }
        }
        void GetGpsCov(double cov[6]) const {
            double sd[3];
            for (int i = 0; i < 3; i++) { sd[i] = m_sdCm[i] / 100.0; cov[i] = sd[i] * sd[i]; }
            const int a[3] = { 0, 0, 1 }, b[3] = { 1, 2, 2 };
            for (int k = 0; k < 3; k++) cov[3 + k] = (m_rho[k] / 127.0) * sd[a[k]] * sd[b[k]];
        }
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
        bool m_swarmCapture = false;
        bool m_hasImuVel = false;
        double m_imuVel[3] = { 0, 0, 0 };
        bool m_hasPps = false;
        uint8_t m_ppsSec = 0;
        uint64_t m_ppsStamp = 0;

        // Range condivisi: solo quelli effettivamente misurati (ID bersaglio -> range [m], eta' [s]).
        // Sul pacchetto: 1 byte = numero di range, poi per ognuno 2 byte ID + 4 byte range in mm
        // + 2 byte eta' in decimi di millisecondo (max 6.5535 s).
        struct SharedRange { double range; double age; bool los; };
        std::map<uint32_t, SharedRange> m_sharedRanges;

        // DS-TWR: 1 byte seq + 5 byte timestamp di trasmissione + 1 byte numero di ricezioni
        // + per ciascuna 2 byte ID + 1 byte (7 bit seq + 1 bit LOS) + 5 byte timestamp di ricezione
        static inline bool s_dsTwr = false;
        static inline bool s_gnssAcc = false;
        uint16_t m_sdCm[3] = { 0, 0, 0 };
        int8_t   m_rho[3]  = { 0, 0, 0 };
        uint8_t  m_seq = 0;
        uint64_t m_uwbTxStamp = 0;
        std::vector<RxReport> m_rxReports;
        std::vector<uint8_t> m_alarmsList;
        std::vector<uint32_t> m_gossipLeaves;
        std::vector<uint32_t> m_gossipEvictions;
    };
} 
#endif