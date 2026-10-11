// Test isolato dell'EKF con misure ritardate.
// Compilazione (una riga):
//   g++ -std=c++20 -I/usr/include/eigen3 tests/test_ekf.cc EKF.cpp -o test_ekf
// Con -DOLD_EKF compila contro un EKF senza il campo 'delay' e stampa solo la traiettoria
// dello stato (serve a dimostrare che con delay = 0 i risultati sono identici).

#include "../EKF.h"
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using Eigen::Vector3d;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { std::printf("  FALLITO: " __VA_ARGS__); std::printf("\n"); g_fail++; } } while (0)

// Ancore fisse attorno al bersaglio (come i droni che condividono i range)
static const std::vector<Vector3d> kAnchors = {
    {110, 100, 50}, {90, 100, 50}, {100, 110, 50}, {100, 90, 50},
    {100, 100, 60}, {100, 100, 40}, {107, 107, 55}, {93, 93, 45}};

// Bersaglio in moto rettilineo uniforme
static Vector3d TruePos(double t) { return Vector3d(100, 100, 50) + Vector3d(7.5, 0, 0) * t; }

// Simula 'steps' frame da 0.1 s. Ogni range condiviso e' stato misurato 'trueDelay' secondi prima.
// Al filtro dico che il ritardo e' 'toldDelay'. Ritorna l'errore medio di posizione negli ultimi 50 frame.
static double Run(double trueDelay, double toldDelay, int steps, unsigned seed, std::FILE* trace)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.10);
    EKF ekf;
    ekf.Init(TruePos(0.0));
    double sumErr = 0; int nErr = 0;
    for (int k = 1; k <= steps; k++) {
        double t = 0.1 * k;
        ekf.Predict(0.1);
        std::vector<EKF::Msmnt> ms;
        for (const Vector3d& a : kAnchors) {
            EKF::Msmnt m;
            m.anchor_pos = a;
            m.is_direct  = false;
            m.is_los     = true;
            m.range      = (TruePos(t - trueDelay) - a).norm() + noise(rng);
#ifndef OLD_EKF
            m.delay      = toldDelay;
#else
            (void)toldDelay;
#endif
            ms.push_back(m);
        }
        ekf.Update(ms);
        if (trace) {
            Eigen::VectorXd s = ekf.GetState();
            for (int i = 0; i < s.size(); i++) std::fprintf(trace, "%.17g ", s(i));
            std::fprintf(trace, "\n");
        }
        if (k > steps - 50) { sumErr += (ekf.GetPosition() - TruePos(t)).norm(); nErr++; }
    }
    return sumErr / nErr;
}

// Consistenza (NEES) con errore GPS delle ancore. Il bersaglio si muove secondo il modello del filtro
// (accelerazione bianca con lo stesso q_acc, discretizzazione esatta): solo cosi' d2 = e' P^-1 e ha media 3.
// Con un bersaglio fermo P risulta conservativa (~0,8x) per il solo mismatch del modello di moto.
// Le ancore seguono il bersaglio (geometria costante). Errore delle ancore: tau < 0 costante, 0 bianco,
// > 0 Gauss-Markov; anchErr = false -> ancore esatte. Ritorna (d2 con solo P, d2 con P + C)
struct D2 { double onlyP, withC; };
static D2 RunAnchorError(double tau, bool anchErr, unsigned seed)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    const Vector3d sig(0.8, 0.8, 1.4);                     // 1 sigma orizz./orizz./vert. dell'ancora
    const Eigen::Matrix3d cov = sig.cwiseProduct(sig).asDiagonal();
    const double dt = 0.1, q = EKF::GetDefaultQAcc();
    Eigen::Matrix2d Qd;
    Qd << q * dt * dt * dt / 3, q * dt * dt / 2, q * dt * dt / 2, q * dt;
    const Eigen::Matrix2d L = Eigen::LLT<Eigen::Matrix2d>(Qd).matrixL();
    const Vector3d c0(100, 100, 50);
    Vector3d p = c0, v = Vector3d::Zero();
    std::vector<Vector3d> e(kAnchors.size());
    for (auto& x : e) x = sig.cwiseProduct(Vector3d(n01(rng), n01(rng), n01(rng)));
    EKF ekf;
    ekf.Init(p + std::sqrt(10.0) * Vector3d(n01(rng), n01(rng), n01(rng)));   // coerente con P0 = 10 I
    for (int k = 1; k <= 300; k++) {
        for (int ax = 0; ax < 3; ax++) {
            Eigen::Vector2d w = L * Eigen::Vector2d(n01(rng), n01(rng));
            p(ax) += v(ax) * dt + w(0); v(ax) += w(1);
        }
        ekf.Predict(dt);
        std::vector<EKF::Msmnt> ms;
        for (size_t i = 0; i < kAnchors.size(); i++) {
            Vector3d a = kAnchors[i] + (p - c0);
            if (anchErr && tau == 0.0) e[i] = sig.cwiseProduct(Vector3d(n01(rng), n01(rng), n01(rng)));
            else if (anchErr && tau > 0.0) {
                double al = std::exp(-dt / tau);
                e[i] = al * e[i] + std::sqrt(1 - al * al) * sig.cwiseProduct(Vector3d(n01(rng), n01(rng), n01(rng)));
            }
            EKF::Msmnt m;
            m.anchor_pos = a + (anchErr ? e[i] : Vector3d::Zero());   // il filtro vede l'ancora col suo errore
            m.is_direct  = false;
            m.is_los     = true;
            m.range      = (p - a).norm() + 0.1 * n01(rng);
            m.uwb_var    = 0.01;                                     // varianza UWB vera (0.1 m)
            if (anchErr) m.anchor_cov = cov;
            ms.push_back(m);
        }
        ekf.Update(ms);
    }
    Vector3d err = ekf.GetPosition() - p;
    Eigen::Matrix3d P = ekf.GetPositionCovariance(), C = ekf.GetAnchorBiasCovariance();
    return { err.dot(P.ldlt().solve(err)), err.dot((P + C).ldlt().solve(err)) };
}

int main(int argc, char** argv)
{
#ifdef OLD_EKF
    // Solo traccia: misure senza ritardo, per il confronto bit per bit con il nuovo EKF
    std::FILE* f = std::fopen(argc > 1 ? argv[1] : "trace_old.txt", "w");
    Run(0.0, 0.0, 300, 7, f);
    std::fclose(f);
    return 0;
#else
    if (argc > 1) {   // traccia per il confronto con il vecchio EKF (che usava q_acc = 0.5)
        EKF::SetDefaultQAcc(0.5);
        std::FILE* f = std::fopen(argv[1], "w");
        Run(0.0, 0.0, 300, 7, f);
        std::fclose(f);
        return 0;
    }

    std::printf("[1] bersaglio a 7.5 m/s, range vecchi di 0.12 s (errore medio di posizione)\n");
    double errNoComp = 0, errComp = 0, errNoDelay = 0, errWrongSign = 0;
    for (unsigned s = 1; s <= 5; s++) {
        errNoDelay   += Run(0.00, 0.00, 300, s, nullptr) / 5;
        errNoComp    += Run(0.12, 0.00, 300, s, nullptr) / 5;
        errComp      += Run(0.12, 0.12, 300, s, nullptr) / 5;
        errWrongSign += Run(0.12, -0.12, 300, s, nullptr) / 5;
    }
    std::printf("  misure senza ritardo (riferimento):  %.3f m\n", errNoDelay);
    std::printf("  ritardo NON compensato:              %.3f m  (atteso ~7.5*0.12 = 0.9 m)\n", errNoComp);
    std::printf("  ritardo compensato:                  %.3f m\n", errComp);
    std::printf("  compensazione con segno sbagliato:   %.3f m\n", errWrongSign);
    CHECK(errNoComp > 0.7, "senza compensazione l'errore dovrebbe essere vicino a 0.9 m");
    CHECK(errComp < 2.0 * errNoDelay + 0.02, "con compensazione l'errore dovrebbe tornare al livello senza ritardo");
    CHECK(errWrongSign > errNoComp, "il segno sbagliato dovrebbe peggiorare le cose");

    std::printf("\n[2] consistenza con errore GPS delle ancore (NEES, 3 g.d.l.: attesi media 3 e 0.1%% oltre 16.27)\n");
    std::printf("  %-20s %12s %12s %13s %13s\n", "ancore", "media d2(P)", "media d2(P+C)", ">16.27 (P)", ">16.27 (P+C)");
    const int N = 1000;
    const double taus[4]    = { 0.0, 0.0, 10.0, -1.0 };
    const bool   errs[4]    = { false, true, true, true };
    const char*  names[4]   = { "esatte", "bianche", "Gauss-Markov 10 s", "costanti" };
    double mP[4], mC[4], tC[4];
    for (int c = 0; c < 4; c++) {
        double sp = 0, sc = 0; int tp = 0, tc = 0;
        for (int s = 1; s <= N; s++) {
            D2 d = RunAnchorError(taus[c], errs[c], 9000 + s);
            sp += d.onlyP / N; sc += d.withC / N; tp += d.onlyP > 16.27; tc += d.withC > 16.27;
        }
        mP[c] = sp; mC[c] = sc; tC[c] = 100.0 * tc / N;
        std::printf("  %-20s %12.2f %12.2f %12.2f%% %12.2f%%\n", names[c], sp, sc, 100.0 * tp / N, tC[c]);
    }
    CHECK(mP[0] > 2.7 && mP[0] < 3.3, "ancore esatte: l'EKF dovrebbe essere consistente (d2 medio ~3)");
    CHECK(mP[1] > 2.7 && mP[1] < 3.3, "ancore bianche: con R_i = R_uwb + u' Sigma u l'EKF dovrebbe essere consistente");
    CHECK(mP[2] > 6.0, "ancore lente: la sola P dovrebbe essere troppo sicura (d2 medio >> 3)");
    CHECK(mC[2] > 2.0 && mC[2] < 3.3 && tC[2] <= 0.3, "ancore lente: P + C dovrebbe essere calibrata o poco conservativa");
    CHECK(mC[3] > 2.0 && mC[3] < 3.3 && tC[3] <= 0.3, "ancore costanti: P + C dovrebbe essere calibrata o poco conservativa");

    std::printf("\n%s\n", g_fail == 0 ? "TUTTI I TEST SUPERATI" : "CI SONO CONTROLLI FALLITI");
    return g_fail == 0 ? 0 : 1;
#endif
}
