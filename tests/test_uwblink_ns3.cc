// Test isolato: l'attenuazione media calcolata con LogDistancePropagationLossModel di ns-3 (UwbLinkNs3.h) coincide con
// uwblink::MeanRxDbm sulla distanza calcolata come in UwbSecurityApp (radice della somma di pow(., 2)):
//   - la distanza di ns-3 (GetDistanceFrom) e' identica bit per bit;
//   - ns-3 calcola EIRP - (L0 + 10 n log10 d) - perdita d'antenna: identico bit per bit (prova che usa la formula attesa);
//   - UwbLink.h calcola (EIRP - L0) - 10 n log10 d - perdita d'antenna: scarto <= 1e-12 dB (ultima cifra).
#include "../include/UwbLinkNs3.h"
#include "ns3/constant-position-mobility-model.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
static int g_fail = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  FALLITO: %s\n", msg); g_fail++; } } while (0)
static bool SameBits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

int main() {
    using namespace ns3;
    uwblink::Params p;
    uwblink::Params q = p; q.n = 2.0; q.sensDbm = -90.0; q.antennaLossDb = 0.0;   // parametri diversi
    auto a = CreateObject<ConstantPositionMobilityModel>();
    auto b = CreateObject<ConstantPositionMobilityModel>();
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> u(-150.0, 150.0), small(-0.6, 0.6), ux(0.0, 1.0);
    for (const auto* par : { &p, &q }) {
        auto loss = uwblink::MakeNs3Loss(*par);
        long n = 0, diff = 0, under1 = 0, distDiff = 0, orderDiff = 0; double maxAbs = 0.0;
        double tx = uwblink::TxEirpDbm(*par), L0 = uwblink::Pl0Db(*par);
        for (int i = 0; i < 1000000; i++) {
            bool close = i % 10 == 0;   // 10% delle coppie a meno di ~1 m (ramo d <= distanza di riferimento)
            Vector pa(u(rng), u(rng), 50.0 + u(rng) / 3.0);
            Vector pb = close ? Vector(pa.x + small(rng), pa.y + small(rng), pa.z + small(rng))
                              : Vector(u(rng), u(rng), 50.0 + u(rng) / 3.0);
            if (i % 1000 == 0) pb = pa;   // distanza nulla
            if (i % 1000 == 1) pb = Vector(pa.x + 1.0, pa.y, pa.z);   // esattamente 1 m
            a->SetPosition(pa); b->SetPosition(pb);
            double d = std::sqrt(std::pow(pa.x - pb.x, 2) + std::pow(pa.y - pb.y, 2) + std::pow(pa.z - pb.z, 2));
            double mine = uwblink::MeanRxDbm(*par, d);
            if (!SameBits(d, a->GetDistanceFrom(b)) || !SameBits(d, b->GetDistanceFrom(a))) distDiff++;
            double nsOrder = tx - (L0 + 10.0 * par->n * std::log10(std::max(d, 1.0))) - par->antennaLossDb;
            if (d <= 1.0) nsOrder = tx - L0 - par->antennaLossDb;
            double ab = uwblink::MeanRxDbmNs3(loss, *par, a, b), ba = uwblink::MeanRxDbmNs3(loss, *par, b, a);
            n++; under1 += d <= 1.0;
            if (!SameBits(mine, ab) || !SameBits(mine, ba)) diff++;
            if (!SameBits(nsOrder, ab) || !SameBits(nsOrder, ba)) orderDiff++;
            maxAbs = std::max(maxAbs, std::max(std::fabs(mine - ab), std::fabs(mine - ba)));
        }
        std::printf("[%s] %ld coppie (%ld entro 1 m): distanze diverse %ld; ordine di ns-3 diverso %ld; "
                    "UwbLink.h diverso nell'ultima cifra %ld, scarto massimo %.3g dB\n",
                    par == &p ? "default" : "n=2, senza perdita d'antenna", n, under1, distDiff, orderDiff, diff, maxAbs);
        CHECK(distDiff == 0, "distanza di ns-3 identica a quella del simulatore");
        CHECK(orderDiff == 0, "ns-3 calcola esattamente EIRP - (L0 + 10 n log10 d) - perdita d'antenna");
        CHECK(maxAbs <= 1e-12, "UwbLink.h e ns-3 entro 1e-12 dB");
    }
    std::printf("\n%s\n", g_fail == 0 ? "TUTTI I TEST SUPERATI" : "CI SONO CONTROLLI FALLITI");
    return g_fail == 0 ? 0 : 1;
}
