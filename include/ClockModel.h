#ifndef CLOCK_MODEL_H
#define CLOCK_MODEL_H

// Orologio di un ricetrasmettitore UWB (stile DW1000/DW3000):
// contatore a 40 bit con tick di 1/(128 * 499.2 MHz) = 15.65 ps, che si azzera ogni ~17.2 s.
// Ogni drone ha un offset casuale (dove si trova il contatore all'avvio) e una deriva (skew) in ppm.
//   tick_locali(t) = (1 + skew) * t / TICK_S + offset + jitter    (modulo 2^40)

#include <cmath>
#include <cstdint>
#include <random>

namespace uwbclock {

constexpr double   TICK_S    = 1.0 / (128.0 * 499.2e6);   // 15.65 ps
constexpr uint64_t WRAP      = (1ULL << 40);              // il contatore e' a 40 bit
constexpr double   LIGHT_C   = 299792458.0;

// Differenza tra due timestamp a 40 bit (b - a), corretta se il contatore si e' azzerato in mezzo
inline int64_t TickDiff(uint64_t a, uint64_t b)
{
    uint64_t d = (b - a) & (WRAP - 1);
    return (d >= WRAP / 2) ? static_cast<int64_t>(d) - static_cast<int64_t>(WRAP)
                           : static_cast<int64_t>(d);
}

class Clock {
public:
    Clock() = default;
    // skewPpm: deriva del quarzo; offsetTicks: valore del contatore a t = 0; jitterS: rumore sul timestamp [s]
    Clock(double skewPpm, uint64_t offsetTicks, double jitterS)
        : m_skew(skewPpm * 1e-6), m_offset(offsetTicks % WRAP), m_jitterS(jitterS) {}

    // Timestamp (tick a 40 bit) che questo orologio registra all'istante globale t [s]
    template <class RNG>
    uint64_t Stamp(double tGlobal, RNG& rng) const
    {
        double ticks = (1.0 + m_skew) * tGlobal / TICK_S;
        if (m_jitterS > 0.0) {
            std::normal_distribution<double> n(0.0, m_jitterS / TICK_S);
            ticks += n(rng);
        }
        // fmod per non perdere precisione con tempi lunghi; poi offset e modulo 2^40
        double wrapped = std::fmod(ticks, static_cast<double>(WRAP));
        if (wrapped < 0) wrapped += static_cast<double>(WRAP);
        uint64_t t = static_cast<uint64_t>(std::floor(wrapped));
        return (t + m_offset) & (WRAP - 1);
    }

    double SkewPpm() const { return m_skew * 1e6; }

private:
    double   m_skew    = 0.0;
    uint64_t m_offset  = 0;
    double   m_jitterS = 0.0;
};

} // namespace uwbclock

#endif
