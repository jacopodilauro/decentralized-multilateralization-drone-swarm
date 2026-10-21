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

    // ---------------------------------------------------------------------------------------------
    // Fase 5: modello realistico (--uwbError=real)
    // ---------------------------------------------------------------------------------------------
    std::printf("[5] real: frazione NLOS = 1 - exp(-d/losDist) con e senza correlazione\n");
    {
        // 400 collegamenti, l'estremo b gira su un cerchio di raggio d attorno ad a a 2 m/s, 0,1 s per passo
        for (double corr : { 0.0, 10.0, 50.0 }) {
            for (double d : { 20.0, 60.0 }) {
                uwberr::Params p; p.on = true; p.corrDist = corr; p.losDist = 100.0;   // NLOS frequente: test piu' stretto
                Ptr<UWBChannel> c = NewChannel(11); c->SetErrorModel(p);
                long nl = 0, tot = 0;
                for (uint32_t l = 0; l < 400; l++)
                    for (int k = 0; k < 600; k++) {
                        double ang = 0.2 * k / d + l;
                        Vector3d a(0, 0, 0), b(d * std::cos(ang), d * std::sin(ang), 0);
                        ChannelCondition q = c->ComputeChannelCondition(2 * l, 2 * l + 1, a, b);
                        nl += !q.is_los; tot++;
                    }
                double f = double(nl) / tot, teo = 1.0 - std::exp(-d / 100.0);
                std::printf("  corr %4.0f m, d %2.0f m: NLOS %.4f (teorica %.4f)\n", corr, d, f, teo);
                CHECK(std::fabs(f - teo) < (corr > 20 ? 0.03 : 0.01), "frazione NLOS con correlazione %.0f m", corr);
            }
        }
    }

    std::printf("[6] real: stesso stato nei due versi (reciprocita')\n");
    {
        uwberr::Params p; p.on = true; p.losDist = 30.0;
        Ptr<UWBChannel> c = NewChannel(5); c->SetErrorModel(p);
        int diff = 0;
        for (int k = 0; k < 2000; k++) {
            Vector3d a(0, 0, 0), b(25.0 + 0.1 * k, 0, 0);
            bool ab = c->ComputeChannelCondition(1, 2, a, b).is_los;
            bool ba = c->ComputeChannelCondition(2, 1, b, a).is_los;   // stesse posizioni: nessuno spostamento
            diff += ab != ba;
        }
        CHECK(diff == 0, "1->2 e 2->1 in stati diversi %d volte", diff);
    }

    std::printf("[7] real: memoria dello stato, indipendente dalla frequenza dei pacchetti\n");
    {
        // estremo b su un cerchio di raggio 20 m: lunghezza media degli episodi NLOS in metri percorsi,
        // con due passi di campionamento diversi (0,2 m e 0,05 m)
        auto run = [](double corr, double step, long& eps, double& frac) {
            uwberr::Params p; p.on = true; p.corrDist = corr; p.losDist = 100.0;
            Ptr<UWBChannel> c = NewChannel(3); c->SetErrorModel(p);
            long episodes = 0, nlosSteps = 0, steps = 0;
            for (uint32_t l = 0; l < 200; l++) {
                bool prev = true;
                for (int k = 0; k < (int)(400.0 / step); k++) {
                    double ang = step * k / 20.0 + l;
                    ChannelCondition q = c->ComputeChannelCondition(2 * l, 2 * l + 1, Vector3d(0, 0, 0),
                                                                    Vector3d(20 * std::cos(ang), 20 * std::sin(ang), 0));
                    if (!q.is_los) { nlosSteps++; if (prev) episodes++; }
                    prev = q.is_los; steps++;
                }
            }
            eps = episodes; frac = double(nlosSteps) / steps;
            return episodes ? step * nlosSteps / episodes : 0.0;
        };
        double prevLen = 0.0;
        for (double corr : { 2.0, 10.0, 50.0 }) {
            long e1, e2; double f1, f2;
            double len1 = run(corr, 0.2, e1, f1), len2 = run(corr, 0.05, e2, f2);
            std::printf("  corr %4.0f m: episodio NLOS medio %.2f m (passo 0,2 m) / %.2f m (passo 0,05 m), NLOS %.3f / %.3f\n",
                        corr, len1, len2, f1, f2);
            CHECK(std::fabs(len1 - len2) < 0.1 * len1, "la durata degli episodi dipende dal passo (corr %.0f m)", corr);
            CHECK(len1 > prevLen, "episodi non piu' lunghi con correlazione maggiore");
            prevLen = len1;
        }
        long e0; double f0;
        double len0 = run(0.0, 0.2, e0, f0);
        double pl = std::exp(-20.0 / 100.0);
        std::printf("  senza correlazione: episodio %.2f m (atteso %.2f m con passo 0,2 m)\n", len0, 0.2 / pl);
        CHECK(std::fabs(len0 - 0.2 / pl) < 0.01, "senza correlazione gli stati non sono indipendenti");
    }

    std::printf("[8] real: rumore LOS che cresce con la distanza (margine dal bilancio della Fase 3)\n");
    {
        uwberr::Params p; p.on = true; p.losDist = 1e12; p.pFa = 0.0;   // solo LOS
        for (double d : { 5.0, 20.0, 50.0, 85.0, 120.0 }) {
            Ptr<UWBChannel> c = NewChannel(9); c->SetErrorModel(p);
            double s1 = 0, s2 = 0; int n = 0;
            for (uint32_t l = 0; l < 50; l++)
                for (int k = 0; k < 2000; k++) {
                    double e = c->ComputeChannelCondition(2 * l, 2 * l + 1, Vector3d(0, 0, 0), Vector3d(d, 0, 0)).ranging_error_m;
                    s1 += e; s2 += e * e; n++;
                }
            double sd = std::sqrt(s2 / n - (s1 / n) * (s1 / n)), teo = std::sqrt(uwberr::MsgVarLos(p, d));
            std::printf("  d %5.1f m: margine %5.1f dB, std %.4f m (teorica %.4f)\n", d, uwberr::MarginDb(p, d), sd, teo);
            CHECK(std::fabs(sd - teo) < 0.01 * teo + 0.001, "std LOS a %.0f m", d);
            CHECK(std::fabs(s1 / n) < 0.003, "media LOS a %.0f m", d);
        }
        uwberr::Params q = p; q.sigmaEdge = 0.0;
        CHECK(std::fabs(uwberr::MsgVarLos(q, 120.0) - 0.01) < 1e-12, "sigmaEdge = 0: varianza non costante");
        // all'85 m del default (margine ~0 dB) la varianza aggiuntiva vale sigmaEdge^2
        CHECK(std::fabs(uwberr::MsgVarLos(p, uwblink::Range50(p.link)) - (0.01 + 0.0625)) < 1e-9, "varianza a margine 0 dB");
    }

    std::printf("[9] real: riconoscimento NLOS imperfetto\n");
    {
        uwberr::Params p; p.on = true; p.losDist = 60.0; p.corrDist = 10.0; p.pMiss = 0.2; p.pFa = 0.07;
        Ptr<UWBChannel> c = NewChannel(21); c->SetErrorModel(p);
        long fa = 0, nLos = 0, missMsg = 0, nNlos = 0;
        int mixedEpisodes = 0, episodes = 0;
        for (uint32_t l = 0; l < 300; l++) {
            bool inEp = false; int epFlagLos = 0, epLen = 0;
            for (int k = 0; k < 1500; k++) {
                double ang = 0.2 * k / 30.0 + l;
                ChannelCondition q = c->ComputeChannelCondition(2 * l, 2 * l + 1, Vector3d(0, 0, 0),
                                                                Vector3d(30 * std::cos(ang), 30 * std::sin(ang), 0));
                if (q.is_los) {
                    nLos++; fa += !q.los_flag;
                    if (inEp) { episodes++; mixedEpisodes += (epFlagLos != 0 && epFlagLos != epLen); inEp = false; }
                } else {
                    nNlos++; missMsg += q.los_flag;
                    if (!inEp) { inEp = true; epFlagLos = 0; epLen = 0; }
                    epFlagLos += q.los_flag; epLen++;
                }
            }
        }
        double pfa = double(fa) / nLos, pm = double(missMsg) / nNlos;
        std::printf("  falsi NLOS %.4f (atteso %.2f); messaggi NLOS non rilevati %.4f (atteso ~%.2f); episodi misti %d su %d\n",
                    pfa, p.pFa, pm, p.pMiss, mixedEpisodes, episodes);
        CHECK(std::fabs(pfa - p.pFa) < 0.003, "falsi NLOS");
        CHECK(std::fabs(pm - p.pMiss) < 0.03, "NLOS non rilevati");
        CHECK(mixedEpisodes == 0, "un episodio e' in parte rilevato e in parte no");
        // rivelatore ideale: stato dichiarato = stato vero
        uwberr::Params r = p; r.pMiss = 0.0; r.pFa = 0.0;
        Ptr<UWBChannel> c2 = NewChannel(21); c2->SetErrorModel(r);
        int wrong = 0;
        for (int k = 0; k < 20000; k++) {
            ChannelCondition q = c2->ComputeChannelCondition(1, 2, Vector3d(0, 0, 0), Vector3d(30 + 0.01 * k, 0, 0));
            wrong += q.los_flag != q.is_los;
        }
        CHECK(wrong == 0, "rivelatore ideale sbaglia %d volte", wrong);
    }

    std::printf("[10] real: bias NLOS persistente nell'episodio e varianza dichiarata ai filtri\n");
    {
        uwberr::Params p; p.on = true; p.losDist = 40.0; p.corrDist = 50.0; p.pFa = 0.0; p.pMiss = 0.0;
        Ptr<UWBChannel> c = NewChannel(8); c->SetErrorModel(p);
        // varianza dentro l'episodio = rumore (non il bias); secondo momento su tutti i messaggi NLOS
        double within = 0, m2 = 0; long nw = 0, n2 = 0;
        for (uint32_t l = 0; l < 300; l++) {
            std::vector<double> ep;
            auto flush = [&]() {
                if (ep.size() >= 5) {
                    double mu = 0; for (double e : ep) mu += e; mu /= ep.size();
                    for (double e : ep) { within += (e - mu) * (e - mu); nw++; }
                }
                ep.clear();
            };
            for (int k = 0; k < 1500; k++) {
                double ang = 0.1 * k / 20.0 + l;
                ChannelCondition q = c->ComputeChannelCondition(2 * l, 2 * l + 1, Vector3d(0, 0, 0),
                                                                Vector3d(20 * std::cos(ang), 20 * std::sin(ang), 0));
                if (!q.is_los) { ep.push_back(q.ranging_error_m); m2 += q.ranging_error_m * q.ranging_error_m; n2++; }
                else flush();
            }
            flush();
        }
        double vw = within / nw, vteo = uwberr::MsgVarLos(p, 20.0) + p.nlosJitter * p.nlosJitter;
        double m2teo = uwberr::BiasSecondMoment(p) + vteo;
        std::printf("  varianza dentro gli episodi %.4f m2 (rumore %.4f); secondo momento NLOS %.3f m2 (teorico %.3f)\n",
                    vw, vteo, m2 / n2, m2teo);
        CHECK(std::fabs(vw - vteo) < 0.05 * vteo, "il bias cambia dentro l'episodio");
        CHECK(std::fabs(m2 / n2 - m2teo) < 0.1 * m2teo, "secondo momento NLOS");
        // DS-TWR: errore = w1 e1 + w2 e2 + w3 e3, w2 = 1/2, w1 + w3 = 1/2. Varianza dichiarata >= vera nei casi estremi
        for (double w1 : { 0.0, 0.25, 0.5 }) {
            double w3 = 0.5 - w1;
            double vLos = (w1 * w1 + 0.25 + w3 * w3) * uwberr::MsgVarLos(p, 20.0);
            double m2Nlos = uwberr::BiasSecondMoment(p) + (w1 * w1 + 0.25 + w3 * w3) * vteo;   // stesso bias sui 3
            CHECK(vLos <= uwberr::DsTwrVar(p, true, 20.0), "varianza LOS dichiarata troppo piccola");
            CHECK(m2Nlos <= uwberr::DsTwrVar(p, false, 20.0), "varianza NLOS dichiarata troppo piccola");
        }
    }

    std::printf("[11] real non tocca il modello storico\n");
    {
        Ptr<UWBChannel> a = NewChannel(42), b = NewChannel(42);
        uwberr::Params off; off.on = false; b->SetErrorModel(off);
        std::vector<double> va = Draw(a, 1, 2, 500), vb = Draw(b, 1, 2, 500);
        CHECK(va == vb, "con on = false il modello storico cambia");
        ChannelCondition k = a->ComputeChannelCondition(3, 4, Vector3d(0, 0, 0), Vector3d(20, 0, 0));
        CHECK(k.los_flag == k.is_los, "storico: stato dichiarato diverso da quello vero");
    }

    std::printf(g_fail == 0 ? "\nTUTTI I TEST SUPERATI\n" : "\n%d CONTROLLI FALLITI\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
