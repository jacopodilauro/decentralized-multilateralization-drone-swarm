// Test isolato di UWBChannel: ogni collegamento (tx, rx) ha il suo generatore casuale.
// Compilazione (una riga):
//   g++ -std=c++20 -I$NS3_PREFIX/include -I/usr/include/eigen3 tests/test_channel.cc UWBChannel.cpp
//       -L$NS3_PREFIX/lib64 -Wl,-rpath,$NS3_PREFIX/lib64 -lns3.44-core -o test_channel

#include "../UWBChannel.h"
#include <cmath>
#include <cstdio>
#include <vector>

static int g_fail = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (!(cond)) { std::printf("  FALLITO: " __VA_ARGS__); std::printf("\n"); g_fail++; } \
    } while (0)

static Ptr<UWBChannel> NewChannel(uint64_t seed)
{
    Ptr<UWBChannel> ch = CreateObject<UWBChannel>();
    ch->SetEnvironment("outdoor");
    ch->SetSeed(seed);
    return ch;
}

static std::vector<double> Draw(Ptr<UWBChannel> ch, uint32_t tx, uint32_t rx, int n, double d = 20.0)
{
    std::vector<double> v;
    for (int i = 0; i < n; i++)
        v.push_back(ch->ComputeChannelCondition(tx, rx, Vector3d(0, 0, 0), Vector3d(d, 0, 0)).ranging_error_m);
    return v;
}

int main()
{
    std::printf("[1] indipendenza tra collegamenti\n");
    {
        Ptr<UWBChannel> a = NewChannel(42), b = NewChannel(42);
        std::vector<double> va = Draw(a, 1, 2, 1000);
        std::vector<double> vb;
        for (int i = 0; i < 1000; i++) {
            Draw(b, 3, 4, 3);                         // altri collegamenti estraggono in mezzo
            Draw(b, 2, 1, 1);
            vb.push_back(Draw(b, 1, 2, 1)[0]);
        }
        CHECK(va == vb, "il rumore su 1->2 cambia se altri collegamenti estraggono");
    }

    std::printf("[2] riproducibilita'\n");
    {
        CHECK(Draw(NewChannel(7), 5, 6, 500) == Draw(NewChannel(7), 5, 6, 500), "stesso seed, sequenze diverse");
        CHECK(Draw(NewChannel(7), 5, 6, 500) != Draw(NewChannel(8), 5, 6, 500), "seed diversi, sequenze uguali");
    }

    std::printf("[3] collegamenti diversi -> sequenze diverse\n");
    {
        Ptr<UWBChannel> c = NewChannel(1);
        CHECK(Draw(c, 1, 2, 200) != Draw(c, 2, 1, 200), "1->2 e 2->1 identici");
        CHECK(Draw(c, 1, 2, 200) != Draw(c, 1, 3, 200), "1->2 e 1->3 identici");
    }

    std::printf("[4] statistica uguale al modello (200k campioni, d = 20 m)\n");
    {
        const double d = 20.0, factor = 1.0 + d / 200.0;
        const int N = 200000;
        Ptr<UWBChannel> c = NewChannel(123);
        int nLos = 0;
        double sumLos = 0, sumLos2 = 0, sumNlos = 0;
        int nNlos = 0;
        for (int i = 0; i < N; i++) {
            uint32_t tx = i % 20, rx = (i / 20) % 20;     // molti collegamenti diversi
            ChannelCondition k = c->ComputeChannelCondition(tx, rx, Vector3d(0, 0, 0), Vector3d(d, 0, 0));
            if (k.is_los) { nLos++; sumLos += k.ranging_error_m; sumLos2 += k.ranging_error_m * k.ranging_error_m; }
            else          { nNlos++; sumNlos += k.ranging_error_m; }
        }
        double pLos = double(nLos) / N, pTeo = std::exp(-d / 500.0);
        double meanLos = sumLos / nLos, stdLos = std::sqrt(sumLos2 / nLos - meanLos * meanLos);
        double meanNlos = sumNlos / nNlos;
        std::printf("  frazione LOS     %.4f (teorica %.4f)\n", pLos, pTeo);
        std::printf("  std errore LOS   %.4f m (teorica %.4f)\n", stdLos, 0.10 * factor);
        std::printf("  media errore NLOS %.4f m (teorica %.4f)\n", meanNlos, 1.4 * factor);
        CHECK(std::fabs(pLos - pTeo) < 0.005, "frazione LOS");
        CHECK(std::fabs(stdLos - 0.10 * factor) < 0.002, "std errore LOS");
        CHECK(std::fabs(meanLos) < 0.002, "media errore LOS");
        CHECK(std::fabs(meanNlos - 1.4 * factor) < 0.05, "media errore NLOS");
    }

    std::printf(g_fail == 0 ? "\nTUTTI I TEST SUPERATI\n" : "\n%d CONTROLLI FALLITI\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
