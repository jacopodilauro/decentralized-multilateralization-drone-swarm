// Test isolato del ricevitore GNSS (nessuna dipendenza da ns-3).
// Compilazione:  g++ -std=c++20 -O2 -I/usr/include/eigen3 tests/test_gnss.cc -o test_gnss

#include "../GnssModel.h"
#include <algorithm>
#include <cstdio>
#include <vector>

using namespace gnss;
using Eigen::Vector3d;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { std::printf("  FALLITO: " __VA_ARGS__); std::printf("\n"); g_fail++; } } while (0)

static double Pct(std::vector<double> v, double q) { std::sort(v.begin(), v.end()); return v[(size_t)(q * (v.size() - 1))]; }

int main()
{
    std::printf("[1] senza errori: la soluzione ai minimi quadrati e' esatta\n");
    {
        Params p; p.commonSigmaM = 0; p.multipathSigM = 0; p.codeNoiseSigM = 0;
        auto c = std::make_shared<Constellation>(p, 1);
        Receiver r(c, 2);
        Vector3d truth(123.4, -56.7, 50.0);
        double maxPos = 0, maxBias = 0;
        for (int k = 0; k < 50; k++) {
            Fix f = r.Measure(truth, 10.0 * k);
            maxPos = std::max(maxPos, (f.pos - truth).norm());
            maxBias = std::max(maxBias, std::fabs(f.clockBiasS - f.trueClockBiasS) * LIGHT_C);
        }
        std::printf("  errore massimo: posizione %.2e m, clock bias %.2e m\n", maxPos, maxBias);
        CHECK(maxPos < 1e-4 && maxBias < 1e-4, "senza errori la soluzione deve essere esatta");
    }

    std::printf("[2] errori tipici: ricevitore fermo, un fix al secondo per un'ora\n");
    Params p;
    auto c = std::make_shared<Constellation>(p, 7);
    Receiver a(c, 11), b(c, 12);
    Vector3d pa(100, 100, 50), pb(120, 100, 50);   // due droni a 20 m
    std::vector<double> absH, absV, relH, relV, biasErr;
    for (int k = 0; k < 3600; k++) {
        double t = k;
        Fix fa = a.Measure(pa, t), fb = b.Measure(pb, t);
        Vector3d ea = fa.pos - pa, eb = fb.pos - pb, rel = ea - eb;
        absH.push_back(ea.head<2>().norm()); absV.push_back(std::fabs(ea.z()));
        relH.push_back(rel.head<2>().norm()); relV.push_back(std::fabs(rel.z()));
        biasErr.push_back(std::fabs(fa.clockBiasS - fa.trueClockBiasS) * LIGHT_C);
    }
    std::printf("  errore ASSOLUTO   orizzontale mediana %.2f m, p95 %.2f m | verticale mediana %.2f m, p95 %.2f m\n",
                Pct(absH, .5), Pct(absH, .95), Pct(absV, .5), Pct(absV, .95));
    std::printf("[3] due ricevitori a 20 m: errore RELATIVO (quello che conta per il rilevamento)\n");
    std::printf("  errore relativo   orizzontale mediana %.2f m, p95 %.2f m | verticale mediana %.2f m, p95 %.2f m\n",
                Pct(relH, .5), Pct(relH, .95), Pct(relV, .5), Pct(relV, .95));
    CHECK(Pct(absH, .5) > 1.0 && Pct(absH, .5) < 6.0, "errore assoluto orizzontale tipico di un GPS singolo (1-6 m)");
    CHECK(Pct(relH, .5) < 0.35 * Pct(absH, .5), "l'errore comune deve cancellarsi tra ricevitori vicini");

    std::printf("[4] clock bias stimato\n");
    std::printf("  errore di stima del clock bias: mediana %.2f m, p95 %.2f m (in distanza, c*dt)\n", Pct(biasErr, .5), Pct(biasErr, .95));
    CHECK(Pct(biasErr, .95) < 15.0, "il clock bias deve essere stimato entro pochi metri");

    std::printf("[5] meno di 4 satelliti: nessuna soluzione\n");
    {
        Params q; q.numSats = 3;
        auto c3 = std::make_shared<Constellation>(q, 3);
        Receiver r3(c3, 4);
        CHECK(!r3.Measure(Vector3d(0, 0, 50), 1.0).ok, "con 3 satelliti il fix non deve essere valido");
    }

    std::printf("\n%s\n", g_fail == 0 ? "TUTTI I TEST SUPERATI" : "CI SONO CONTROLLI FALLITI");
    return g_fail == 0 ? 0 : 1;
}
