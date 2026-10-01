// Test isolato di UwbHeader: un header scritto in un pacchetto e riletto deve restituire
// esattamente gli stessi campi. Compilato ed eseguito da Claude prima di ogni patch che tocca
// il formato dei pacchetti.
//
// Compilazione (fuori da ns-3, con ns-3 installato da pip):
//   g++ -std=c++20 -I$NS3_PREFIX/include tests/test_header.cc UwbHeader.cpp
//       -L$NS3_PREFIX/lib64 -Wl,-rpath,$NS3_PREFIX/lib64 -lns3.44-network -lns3.44-core -o test_header
//   (tutto su una riga)

#include "../UwbHeader.h"
#include "ns3/packet.h"
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ns3;

static int g_fail = 0;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) { std::printf("  FALLITO: %s\n", msg); g_fail++; }    \
    } while (0)

// Il valore che deve arrivare al ricevitore: range troncato al millimetro (come nel formato originale)
static double Quantized(double r) { return static_cast<double>(static_cast<uint32_t>(r * 1000.0)) / 1000.0; }

static UwbHeader RoundTrip(const UwbHeader& in, uint32_t* bytesOnWire)
{
    Ptr<Packet> p = Create<Packet>();
    p->AddHeader(in);
    *bytesOnWire = p->GetSize();
    UwbHeader out;
    p->RemoveHeader(out);
    CHECK(p->GetSize() == 0, "dopo RemoveHeader il pacchetto deve essere vuoto (byte letti == byte scritti)");
    return out;
}

static void TestCase(const char* name, const std::vector<std::pair<uint32_t, double>>& ranges,
                     const std::vector<uint32_t>& leaves, const std::vector<uint32_t>& evictions,
                     const std::vector<uint8_t>& alarms)
{
    std::printf("[%s]\n", name);
    UwbHeader h;
    h.SetSenderId(7);
    h.SetTxTimestampPs(123456789012ULL);
    h.SetGpsPosition(100.123456789, -3.5, 50.25);
    h.SetImLeaving(true);
    h.SetGossipLeaves(leaves);
    h.SetGossipEvictions(evictions);
    h.SetAlarmsList(alarms);
    for (auto& [id, r] : ranges) h.SetSharedRange(id, r);

    uint32_t wire = 0;
    UwbHeader o = RoundTrip(h, &wire);
    CHECK(wire == h.GetSerializedSize(), "byte sul pacchetto == GetSerializedSize()");
    CHECK(o.GetSenderId() == 7, "senderId");
    CHECK(o.GetTxTimestampPs() == 123456789012ULL, "timestamp");
    CHECK(o.GetGpsX() == 100.123456789 && o.GetGpsY() == -3.5 && o.GetGpsZ() == 50.25, "GPS esatto");
    CHECK(o.GetImLeaving() == true, "flag leave");
    CHECK(o.GetGossipLeaves() == leaves, "gossip leave");
    CHECK(o.GetGossipEvictions() == evictions, "gossip eviction");
    CHECK(o.GetAlarmsList() == alarms, "bitmap allarmi");

    // Ogni range positivo deve arrivare quantizzato al mm; quelli negativi (o mai impostati) come -1
    for (auto& [id, r] : ranges) {
        double got = o.GetSharedRange(id);
        double exp = (r < 0.0) ? -1.0 : Quantized(r);
        char msg[128];
        std::snprintf(msg, sizeof msg, "range verso %u: atteso %.3f, ricevuto %.3f", id, exp, got);
        CHECK(got == exp, msg);
    }
    CHECK(o.GetSharedRange(99999) == -1.0, "ID mai impostato -> -1");
    std::printf("  %u byte sul pacchetto\n", wire);
}

int main()
{
    TestCase("nessun range", {}, {}, {}, {});
    TestCase("10 range, ID 0-19 (caso tipico)",
             {{1, 9.8765}, {2, 10.0}, {3, 0.001}, {5, 12.3456}, {8, 15.5}, {11, 20.0},
              {13, 7.25}, {16, 3.999}, {18, 1000.0}, {19, 42.4242}},
             {10, 15}, {4}, {0x05, 0x00, 0x80});
    TestCase("ID alti (300, 65535)", {{300, 5.5}, {65535, 1.25}}, {}, {}, {});
    TestCase("range negativo = nessuna misura", {{4, -1.0}, {6, 2.5}}, {}, {}, {0x01});

    // Eta' dei range: andata e ritorno con risoluzione 0.1 ms, saturazione a 6.5535 s, default 0
    {
        std::printf("[eta' dei range]\n");
        UwbHeader h;
        h.SetSharedRange(1, 5.0, 0.12345);   // -> 0.1235 (arrotondata al decimo di ms)
        h.SetSharedRange(2, 6.0, 0.0);
        h.SetSharedRange(3, 7.0, 100.0);     // oltre il massimo -> 6.5535
        h.SetSharedRange(4, 8.0);            // default 0
        uint32_t w = 0;
        UwbHeader o = RoundTrip(h, &w);
        CHECK(std::fabs(o.GetSharedRangeAge(1) - 0.1235) < 1e-9, "eta' 0.12345 s -> 0.1235 s");
        CHECK(o.GetSharedRangeAge(2) == 0.0, "eta' 0");
        CHECK(std::fabs(o.GetSharedRangeAge(3) - 6.5535) < 1e-9, "eta' saturata a 6.5535 s");
        CHECK(o.GetSharedRangeAge(4) == 0.0, "eta' di default 0");
        CHECK(o.GetSharedRangeAge(99) == -1.0, "eta' di un ID assente = -1");
        CHECK(o.GetSharedRange(1) == 5.0 && o.GetSharedRange(3) == 7.0, "range invariati");
    }

    // Sovrascrittura: l'ultimo valore impostato vince
    {
        std::printf("[sovrascrittura dello stesso ID]\n");
        UwbHeader h;
        h.SetSharedRange(3, 1.0);
        h.SetSharedRange(3, 2.0);
        uint32_t w = 0;
        UwbHeader o = RoundTrip(h, &w);
        CHECK(o.GetSharedRange(3) == 2.0, "vince l'ultimo valore");
    }

    // Dimensione: il blocco dei range dipende solo da quanti range ci sono, non dall'ID piu' alto
    {
        std::printf("[dimensione indipendente dall'ID massimo]\n");
        UwbHeader a, b;
        a.SetSharedRange(1, 5.0);
        b.SetSharedRange(60000, 5.0);
        CHECK(a.GetSerializedSize() == b.GetSerializedSize(), "stessa dimensione con ID 1 o ID 60000");
        std::printf("  1 range: %u byte (ID 1) e %u byte (ID 60000)\n", a.GetSerializedSize(), b.GetSerializedSize());
    }

    std::printf(g_fail == 0 ? "\nTUTTI I TEST SUPERATI\n" : "\n%d CONTROLLI FALLITI\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
