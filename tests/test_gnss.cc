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

// Formazione come nel simulatore: un drone al centro e gli altri su una sfera di raggio r (Fibonacci)
static std::vector<Vector3d> Formation(const Vector3d& c, double r, int n)
{
    std::vector<Vector3d> p{ c };
    for (int k = 0; k < n - 1; k++) {
        double z = 1.0 - 2.0 * (k + 0.5) / (n - 1), rr = std::sqrt(1.0 - z * z), phi = k * 2.39996323;
        p.push_back(c + r * Vector3d(rr * std::cos(phi), rr * std::sin(phi), z));
    }
    return p;
}

// TDoA ai minimi quadrati: d[i] = |p_i - s| - |p_0 - s|, i >= 1. Gauss-Newton da 12 direzioni a 300 m,
// tiene la soluzione col residuo minimo. Distanza limitata a 20 km (base corta: la distanza e' mal definita)
static Vector3d SolveTdoa(const std::vector<Vector3d>& p, const std::vector<double>& d)
{
    Vector3d best = p[0]; double bestRes = 1e300;
    for (int a = 0; a < 12; a++) {
        Vector3d s = p[0] + 300.0 * Vector3d(std::cos(a * M_PI / 6), std::sin(a * M_PI / 6), -0.1);
        for (int it = 0; it < 30; it++) {
            Eigen::MatrixXd H(p.size() - 1, 3); Eigen::VectorXd r(p.size() - 1);
            Vector3d u0 = (s - p[0]).normalized();
            for (size_t i = 1; i < p.size(); i++) {
                Vector3d ui = (s - p[i]).normalized();
                H.row(i - 1) = (ui - u0).transpose();
                r(i - 1) = d[i] - ((s - p[i]).norm() - (s - p[0]).norm());
            }
            Vector3d dx = (H.transpose() * H + 1e-9 * Eigen::Matrix3d::Identity()).ldlt().solve(H.transpose() * r);
            s += dx;
            if ((s - p[0]).norm() > 20000.0) s = p[0] + 20000.0 * (s - p[0]).normalized();
            if (dx.norm() < 1e-3) break;
        }
        double res = 0;
        for (size_t i = 1; i < p.size(); i++) { double e = d[i] - ((s - p[i]).norm() - (s - p[0]).norm()); res += e * e; }
        if (res < bestRes) { bestRes = res; best = s; }
    }
    return best;
}

// Una prova: N droni fermi catturati dallo spoofer per T secondi, PPS a 1 Hz con jitter ppsSig [s],
// differenze di PPS mediate nel tempo e corrette con gli offset UWB (qui ideali). Errori di direzione [gradi]
// e di distanza [m] della stima dello spoofer
struct LocErr { double azDeg, elDeg, rangeM; };
static LocErr SpooferTrial(const Params& prm, double radius, int nDrones, double T, double ppsSig, uint64_t seed)
{
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    auto c = std::make_shared<Constellation>(prm, seed * 7 + 1);
    auto sp = std::make_shared<Spoofer>();
    const Vector3d center(100, 100, 50);
    sp->antenna  = center + Vector3d(400, -300, -48);     // a terra (z = 2 m), a 500 m in orizzontale
    sp->fakePos0 = center + Vector3d(0, 15, 0);
    sp->timeOffsetS = 2e-6;
    sp->startS = 0.0;
    std::vector<Vector3d> p = Formation(center, radius, nDrones);
    std::vector<Receiver> rx;
    for (int i = 0; i < nDrones; i++) { rx.emplace_back(c, seed * 131 + i); rx.back().SetSpoofer(sp); }
    std::vector<double> d(nDrones, 0.0);
    int K = (int)T;
    for (int k = 0; k < K; k++) {
        double t = 1.0 * k, pps0 = 0;
        for (int i = 0; i < nDrones; i++) {
            double pps = rx[i].Measure(p[i], t).ppsErrorS + ppsSig * n01(rng);
            if (i == 0) pps0 = pps;
            d[i] += LIGHT_C * (pps - pps0) / K;
        }
    }
    Vector3d s = SolveTdoa(p, d), v = s - center, vt = sp->antenna - center;
    double az  = std::atan2(v.y(), v.x()), azt = std::atan2(vt.y(), vt.x());
    double daz = std::fabs(std::remainder(az - azt, 2 * M_PI)) * 180 / M_PI;
    double el  = std::atan2(v.z(), v.head<2>().norm()), elt = std::atan2(vt.z(), vt.head<2>().norm());
    return { daz, std::fabs(el - elt) * 180 / M_PI, std::fabs(v.norm() - vt.norm()) };
}

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

    std::printf("[6] covarianza dichiarata dell'errore individuale (30 costellazioni, d2 con 3 g.d.l.)\n");
    {
        double sA = 0, sB = 0, qA = 0, qB = 0; int n = 0;
        for (int cs = 0; cs < 30; cs++) {
            Params q; q.commonSigmaM = 0.0;                    // solo errore individuale
            auto cc = std::make_shared<Constellation>(q, 31337 + cs);
            Receiver r(cc, 99 + cs);
            for (int k = 0; k < 300; k++) {                    // un campione al secondo (multipath tau 10 s)
                Fix f = r.Measure(Vector3d(0, 0, 0), 1.0 * k);
                Vector3d e = f.pos;
                Eigen::Matrix3d D = Vector3d(f.hAccM * f.hAccM, f.hAccM * f.hAccM, f.vAccM * f.vAccM).asDiagonal();
                double a = e.dot(D.ldlt().solve(e)), b = e.dot(f.covInd.ldlt().solve(e));
                sA += a; sB += b; qA += a > 16.27; qB += b > 16.27; n++;
            }
        }
        std::printf("  diag(h,h,v):       media d2 %.2f, oltre 16.27 %.2f%%\n", sA / n, 100 * qA / n);
        std::printf("  covarianza piena:  media d2 %.2f, oltre 16.27 %.2f%% (attesi 3 e 0.1%%)\n", sB / n, 100 * qB / n);
        CHECK(sB / n > 2.7 && sB / n < 3.3 && 100 * qB / n < 0.25, "la covarianza piena deve essere calibrata");
        CHECK(qA > 2 * qB, "diag(h,h,v) perde l'anisotropia: coda piu' pesante");
    }

    std::printf("[7] spoofer esterno a singola antenna (8 droni, antenna a terra a 500 m, posizione falsa +15 m in y)\n");
    {
        Params q;
        auto cc = std::make_shared<Constellation>(q, 2024);
        auto sp = std::make_shared<Spoofer>();
        const Vector3d center(100, 100, 50);
        sp->antenna = center + Vector3d(400, -300, -48);
        sp->fakePos0 = center + Vector3d(0, 15, 0);
        sp->timeOffsetS = 2e-6;
        sp->startS = 30.0;
        std::vector<Vector3d> p = Formation(center, 10.0, 8);
        std::vector<Receiver> rx;
        for (int i = 0; i < 8; i++) { rx.emplace_back(cc, 600 + i); rx.back().SetSpoofer(sp); }
        double maxPpsBefore = 0, maxPosErrBefore = 0, maxFake = 0, ss = 0; int nRes = 0; bool allSpoofed = true;
        for (int k = 0; k < 900; k++) {
            double t = 0.1 * k;
            std::vector<Fix> f;
            for (int i = 0; i < 8; i++) f.push_back(rx[i].Measure(p[i], t));
            for (int i = 0; i < 8; i++) {
                if (t < 30.0) {
                    maxPpsBefore    = std::max(maxPpsBefore, std::fabs(f[i].ppsErrorS) * LIGHT_C);
                    maxPosErrBefore = std::max(maxPosErrBefore, (f[i].pos - p[i]).norm());
                } else {
                    allSpoofed = allSpoofed && f[i].spoofed;
                    maxFake = std::max(maxFake, (f[i].pos - sp->FakePos(t)).norm());
                    if (i > 0) {   // TDoA: differenza di PPS tra droni contro differenza di distanza vera
                        double meas = LIGHT_C * (f[i].ppsErrorS - f[0].ppsErrorS);
                        double tru  = (p[i] - sp->antenna).norm() - (p[0] - sp->antenna).norm();
                        ss += (meas - tru) * (meas - tru); nRes++;
                    }
                }
            }
        }
        double rms = std::sqrt(ss / nRes);
        std::printf("  prima: errore PPS max %.1f m (errore comune), errore di posizione max %.1f m\n", maxPpsBefore, maxPosErrBefore);
        std::printf("  dopo:  distanza max dalla posizione falsa %.2f m; TDoA dai PPS - verita' RMS %.2f m\n", maxFake, rms);
        CHECK(maxPpsBefore < 20.0 && maxPosErrBefore < 20.0, "prima dell'attivazione il ricevitore e' normale");
        CHECK(allSpoofed, "dopo l'attivazione tutti i ricevitori sono catturati");
        CHECK(maxFake < 1.5, "tutti i droni vedono la stessa posizione falsa (una sola antenna)");
        CHECK(rms < 1.5, "le differenze di PPS sono le differenze di distanza dallo spoofer (TDoA)");

        Params z; z.commonSigmaM = 0; z.multipathSigM = 0; z.codeNoiseSigM = 0; z.spoofMultipathSigM = 0;
        LocErr e = SpooferTrial(z, 10.0, 8, 10.0, 0.0, 5);
        std::printf("  senza rumore: errore di azimut %.4f gradi, di distanza %.3f m\n", e.azDeg, e.rangeM);
        CHECK(e.azDeg < 0.01 && e.rangeM < 1.0, "senza rumore la TDoA deve ritrovare l'antenna");
    }

    std::printf("[8] (informativo) localizzazione dello spoofer a 500 m dai PPS, 60 s a 1 Hz, offset UWB ideali,\n"
                "    200 prove: mediana / p95 degli errori\n");
    {
        Params q;
        std::printf("  %-26s %-10s %18s %18s %22s\n", "formazione", "jitter PPS", "azimut [gradi]", "elevazione [gradi]", "distanza [m]");
        for (double radius : { 10.0, 50.0 })
            for (double js : { 0.0, 5e-9, 20e-9 }) {
                std::vector<double> az, el, rg;
                for (int s = 1; s <= 200; s++) {
                    LocErr e = SpooferTrial(q, radius, 8, 60.0, js, 1000 + s);
                    az.push_back(e.azDeg); el.push_back(e.elDeg); rg.push_back(e.rangeM);
                }
                char f[32]; std::snprintf(f, sizeof f, "8 droni, raggio %2.0f m", radius);
                std::printf("  %-26s %7.0f ns %8.2f / %-7.2f %8.2f / %-7.2f %10.0f / %-9.0f\n", f, js * 1e9,
                            Pct(az, .5), Pct(az, .95), Pct(el, .5), Pct(el, .95), Pct(rg, .5), Pct(rg, .95));
            }
    }

    std::printf("\n%s\n", g_fail == 0 ? "TUTTI I TEST SUPERATI" : "CI SONO CONTROLLI FALLITI");
    return g_fail == 0 ? 0 : 1;
}
