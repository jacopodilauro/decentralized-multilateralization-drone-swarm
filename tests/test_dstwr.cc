// Test isolato del modello di orologio e del DS-TWR (nessuna dipendenza da ns-3).
// Compilazione:  g++ -std=c++20 -O2 -I/usr/include/eigen3 tests/test_dstwr.cc -o test_dstwr

#include "../DsTwr.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <vector>

using namespace uwbclock;
using Eigen::Vector3d;
using Traj = std::function<Vector3d(double)>;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { std::printf("  FALLITO: " __VA_ARGS__); std::printf("\n"); g_fail++; } } while (0)

// Istante di ricezione: il segnale parte da tx all'istante tTx e arriva al ricevitore in movimento
static double RxTime(const Traj& tx, const Traj& rx, double tTx)
{
    double t = tTx + (rx(tTx) - tx(tTx)).norm() / LIGHT_C;
    for (int i = 0; i < 3; i++) t = tTx + (rx(t) - tx(tTx)).norm() / LIGHT_C;
    return t;
}

struct Exchange { DsTwrStamps s; double tA1, tB2, tA3, tB3; };

// Scambio TDMA: A trasmette a tA1, B nel suo slot dopo 'gap' secondi, A di nuovo dopo un frame
static Exchange Run(const Traj& pA, const Traj& pB, const Clock& cA, const Clock& cB,
                    double tA1, double gap, double frame, std::mt19937& rng)
{
    Exchange e;
    e.tA1 = tA1; e.tB2 = tA1 + gap; e.tA3 = tA1 + frame;
    double tB1 = RxTime(pA, pB, e.tA1);
    double tA2 = RxTime(pB, pA, e.tB2);
    e.tB3 = RxTime(pA, pB, e.tA3);
    e.s = { cA.Stamp(e.tA1, rng), cA.Stamp(tA2, rng), cA.Stamp(e.tA3, rng),
            cB.Stamp(tB1, rng),   cB.Stamp(e.tB2, rng), cB.Stamp(e.tB3, rng) };
    return e;
}

static Clock RandomClock(std::mt19937& rng, double maxPpm, double jitter)
{
    std::uniform_real_distribution<double> ppm(-maxPpm, maxPpm);
    std::uniform_int_distribution<uint64_t> off(0, WRAP - 1);
    return Clock(ppm(rng), off(rng), jitter);
}

int main()
{
    std::mt19937 rng(2026);
    const double FRAME = 0.1;   // frame TDMA attuale: 20 slot da 5 ms

    std::printf("[1] droni fermi a 15 m, orologi con offset casuale e skew +-20 ppm\n");
    {
        Traj pA = [](double) { return Vector3d(0, 0, 0); };
        Traj pB = [](double) { return Vector3d(15, 0, 0); };
        double maxDs = 0, sum2Ds = 0, minOneWay = 1e30; int n = 2000;
        std::uniform_real_distribution<double> start(1.0, 200.0), gapU(0.005, 0.095);
        for (int i = 0; i < n; i++) {
            Clock cA = RandomClock(rng, 20, 0.0), cB = RandomClock(rng, 20, 0.0);
            Exchange e = Run(pA, pB, cA, cB, start(rng), gapU(rng), FRAME, rng);
            double err = DsTwrRange(e.s) - 15.0;
            maxDs = std::max(maxDs, std::fabs(err)); sum2Ds += err * err;
            // metodo attuale: ToA a una via = (ricezione su B - trasmissione su A) * c
            double oneWay = TickDiff(e.s.tA1, e.s.tB1) * TICK_S * LIGHT_C - 15.0;
            minOneWay = std::min(minOneWay, std::fabs(oneWay));
        }
        std::printf("  DS-TWR:          errore RMS %.2f mm, massimo %.2f mm\n", 1e3 * std::sqrt(sum2Ds / n), 1e3 * maxDs);
        std::printf("  ToA a una via:   errore MINIMO su %d prove %.0f m\n", n, minOneWay);
        CHECK(maxDs < 0.01, "DS-TWR con droni fermi dovrebbe sbagliare meno di 1 cm");
        CHECK(minOneWay > 1.0, "il ToA a una via con orologi non sincronizzati dovrebbe essere inutilizzabile");
    }

    std::printf("[2] con jitter dei timestamp (10 ps)\n");
    {
        Traj pA = [](double) { return Vector3d(0, 0, 0); };
        Traj pB = [](double) { return Vector3d(15, 0, 0); };
        double sum2 = 0; int n = 2000;
        for (int i = 0; i < n; i++) {
            Clock cA = RandomClock(rng, 20, 10e-12), cB = RandomClock(rng, 20, 10e-12);
            double err = DsTwrRange(Run(pA, pB, cA, cB, 5.0 + i * 0.01, 0.05, FRAME, rng).s) - 15.0;
            sum2 += err * err;
        }
        std::printf("  errore RMS %.2f mm\n", 1e3 * std::sqrt(sum2 / n));
        CHECK(std::sqrt(sum2 / n) < 0.01, "con 10 ps di jitter l'errore RMS dovrebbe restare sotto 1 cm");
    }

    std::printf("[3] scambio a cavallo dell'azzeramento del contatore a 40 bit\n");
    {
        Traj pA = [](double) { return Vector3d(0, 0, 0); };
        Traj pB = [](double) { return Vector3d(15, 0, 0); };
        // offset scelti perche' il contatore di A e di B si azzerino durante lo scambio
        double tStart = 3.0;
        uint64_t nearWrap = WRAP - static_cast<uint64_t>((tStart + 0.03) / TICK_S);
        Clock cA(12.0, nearWrap, 0.0), cB(-17.0, nearWrap + 1000, 0.0);
        Exchange e = Run(pA, pB, cA, cB, tStart, 0.05, FRAME, rng);
        bool wrapped = (e.s.tA3 < e.s.tA1) || (e.s.tB3 < e.s.tB1);
        double err = DsTwrRange(e.s) - 15.0;
        std::printf("  contatore azzerato durante lo scambio: %s, errore %.2f mm\n", wrapped ? "si" : "NO", 1e3 * err);
        CHECK(wrapped, "il test dovrebbe davvero attraversare l'azzeramento");
        CHECK(std::fabs(err) < 0.01, "l'azzeramento del contatore non deve rompere la misura");
    }

    std::printf("[4] droni in movimento: errore rispetto alla distanza vera in tre istanti\n");
    std::printf("      caso                              inizio    meta'    centrale   fine   (mediana |errore|, cm)\n");
    struct Case { const char* name; Traj pB; };
    Traj pA = [](double) { return Vector3d(0, 0, 50); };   // drone base fermo
    std::vector<Case> cases = {
        {"radiale 1 m/s (drone base)",         [](double t) { return Vector3d(10 + 1.0 * t, 0, 50); }},
        {"radiale 7.5 m/s",                    [](double t) { return Vector3d(10 + 7.5 * t, 0, 50); }},
        {"radiale 15 m/s",                     [](double t) { return Vector3d(10 + 15.0 * t, 0, 50); }},
        {"ospite in orbita r=15 m, 7.5 m/s",   [](double t) { return Vector3d(5 + 15 * std::cos(0.5 * t), 15 * std::sin(0.5 * t), 50); }},
    };
    double midErrWorst = 0;
    for (auto& c : cases) {
        std::vector<double> eS, eM, eC, eE;
        std::uniform_real_distribution<double> start(1.0, 50.0), gapU(0.005, 0.095);
        for (int i = 0; i < 2000; i++) {
            Clock cA = RandomClock(rng, 20, 10e-12), cB = RandomClock(rng, 20, 10e-12);
            Exchange e = Run(pA, c.pB, cA, cB, start(rng), gapU(rng), FRAME, rng);
            double r = DsTwrRange(e.s);
            double tMid = 0.5 * (e.tA1 + e.tB3);
            eS.push_back(std::fabs(r - (c.pB(e.tA1) - pA(e.tA1)).norm()));
            eM.push_back(std::fabs(r - (c.pB(tMid) - pA(tMid)).norm()));
            eC.push_back(std::fabs(r - (c.pB(e.tB2) - pA(e.tB2)).norm()));   // messaggio centrale
            eE.push_back(std::fabs(r - (c.pB(e.tB3) - pA(e.tB3)).norm()));
        }
        auto med = [](std::vector<double> v) { std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end()); return v[v.size() / 2]; };
        std::printf("      %-34s %7.2f  %7.2f  %8.2f  %7.2f\n", c.name, 100 * med(eS), 100 * med(eM), 100 * med(eC), 100 * med(eE));
        midErrWorst = std::max(midErrWorst, med(eC));
    }
    // Al primo ordine il DS-TWR restituisce la distanza all'istante del messaggio centrale (tB2)
    CHECK(midErrWorst < 0.01, "riferita al messaggio centrale la misura dovrebbe sbagliare meno di 1 cm");

    std::printf("\n%s\n", g_fail == 0 ? "TUTTI I TEST SUPERATI" : "CI SONO CONTROLLI FALLITI");
    return g_fail == 0 ? 0 : 1;
}
