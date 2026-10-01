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

int main(int argc, char** argv)
{
#ifdef OLD_EKF
    // Solo traccia: misure senza ritardo, per il confronto bit per bit con il nuovo EKF
    std::FILE* f = std::fopen(argc > 1 ? argv[1] : "trace_old.txt", "w");
    Run(0.0, 0.0, 300, 7, f);
    std::fclose(f);
    return 0;
#else
    if (argc > 1) {   // traccia per il confronto con il vecchio EKF
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

    std::printf("\n%s\n", g_fail == 0 ? "TUTTI I TEST SUPERATI" : "CI SONO CONTROLLI FALLITI");
    return g_fail == 0 ? 0 : 1;
#endif
}
