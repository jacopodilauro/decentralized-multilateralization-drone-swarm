// Test isolato del collegamento radio UWB (Fase 3): numeri del bilancio di collegamento e perdite simulate
#include "../UwbLink.h"
#include <cstdio>
#include <random>
static int g_fail = 0;
#define CHECK(c, msg) do { if (!(c)) { std::printf("  FALLITO: %s\n", msg); g_fail++; } } while (0)

static double Fraction(const uwblink::Params& p, double d, int n, unsigned seed) {
    std::mt19937_64 rng(seed); int ok = 0;
    for (int i = 0; i < n; i++) ok += uwblink::Received(p, d, rng);
    return double(ok) / n;
}

int main() {
    uwblink::Params p;
    std::printf("[1] bilancio di collegamento (default)\n");
    double eirp = uwblink::TxEirpDbm(p), pl0 = uwblink::Pl0Db(p), r50 = uwblink::Range50(p);
    std::printf("  EIRP %.2f dBm, attenuazione a 1 m %.2f dB, ricezione al 50%% a %.1f m\n", eirp, pl0, r50);
    CHECK(std::fabs(eirp + 14.32) < 0.05, "EIRP = -41,3 dBm/MHz su 499,2 MHz = -14,3 dBm");
    CHECK(std::fabs(pl0 - 48.69) < 0.05, "spazio libero a 1 m a 6,4896 GHz = 48,7 dB");
    CHECK(r50 > 80 && r50 < 90, "portata al 50% ~85 m con sensibilita' -100 dBm");
    CHECK(std::fabs(uwblink::Per(p, p.sensDbm) - 0.5) < 1e-12, "PER = 50% alla sensibilita'");
    CHECK(uwblink::Per(p, p.sensDbm + 3) < uwblink::Per(p, p.sensDbm) &&
          uwblink::Per(p, p.sensDbm) < uwblink::Per(p, p.sensDbm - 3), "PER decrescente con la potenza");
    uwblink::Params q = p; q.sensDbm = -90.0;
    std::printf("  con sensibilita' -90 dBm: ricezione al 50%% a %.1f m\n", uwblink::Range50(q));
    CHECK(uwblink::Range50(q) > 20 && uwblink::Range50(q) < 26, "portata pessimistica ~23 m");

    std::printf("[2] pacchetti ricevuti (Monte Carlo, 20000 per distanza)\n");
    double f30 = Fraction(p, 30, 20000, 1), f50 = Fraction(p, r50, 20000, 2), f2 = Fraction(p, 2 * r50, 20000, 3);
    std::printf("  30 m: %.4f | %.0f m: %.3f | %.0f m: %.4f\n", f30, r50, f50, 2 * r50, f2);
    CHECK(f30 > 0.99, "a 30 m quasi tutti i pacchetti arrivano");
    CHECK(f50 > 0.45 && f50 < 0.55, "alla portata al 50% ne arriva circa la meta'");
    CHECK(f2 < 0.01, "al doppio della portata quasi nessuno");

    std::printf("\n%s\n", g_fail == 0 ? "TUTTI I TEST SUPERATI" : "CI SONO CONTROLLI FALLITI");
    return g_fail == 0 ? 0 : 1;
}
