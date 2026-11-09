#pragma once
// Uscita a terminale del simulatore (in inglese) e riepilogo per gli script. Solo stampa: non tocca la simulazione.
//   - intestazione con la configurazione, poi un evento per riga ("  101.1 s  CAPTURE  ...");
//   - eventi simultanei dello stesso tipo raggruppati in una riga (Group);
//   - su terminale, una riga di avanzamento fissa in fondo (spinner, barra, percentuale, tempo simulato, ETA),
//     ridisegnata dopo ogni evento; con l'uscita su file niente barra, niente colori, niente caratteri di controllo;
//   - colori solo su terminale e senza NO_COLOR; Unicode solo con una locale UTF-8 (altrimenti ASCII);
//   - alla fine: riga di chiusura, riepilogo e summary.json con i valori che gli script leggevano da stdout.txt.
// Livelli: 0 = intestazione e riepilogo, 1 = eventi principali, 2 = dettagli e debug (opzione --verbose).
#include <unistd.h>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ui {

struct State {
    bool tty = false, color = false, unicode = false;
    int verbose = 1;
    double total = 0.0, now = 0.0;
    std::chrono::steady_clock::time_point start, lastDraw;
    bool drawn = false;
    int spin = 0;
    // gruppi di eventi in attesa (piu' tipi insieme: eventi simultanei di tipo diverso non si spezzano a vicenda)
    struct Grp { double t; std::string tag, templ; std::vector<int> ids; };
    std::vector<Grp> groups;
    // intestazione, riepilogo, summary.json
    std::string title;
    std::vector<std::pair<std::string, std::string>> config, summary, json;
    // scenario (dalla configurazione): chi mente e da quando, quando parte lo spoofer
    std::set<int> insiders;
    double attackS = -1.0, spoofS = -1.0;
    // allarmi collettivi: osservatori che oggi accusano ogni drone, episodi, prime scoperte degli interni
    struct Ep { int target; double start, end; bool insider, onset; };
    std::map<int, std::set<int>> accusers;
    std::map<int, size_t> openEp;          // drone -> indice dell'episodio aperto
    std::vector<Ep> episodes;
    std::map<int, double> detected;        // interno -> primo allarme collettivo dopo l'attacco
    double firstConsensus = -1.0;
    // errore di stima (dalle righe del CSV, solo letto): recente per la barra, prima dell'attacco per il riepilogo
    std::vector<float> recentErr, preErr;
    size_t recentPos = 0;
};
inline State& S() { static State s; return s; }

inline bool Utf8Locale() {
    for (const char* v : { "LC_ALL", "LC_CTYPE", "LANG" }) {
        const char* e = std::getenv(v);
        if (e && *e) {
            std::string x(e);
            std::transform(x.begin(), x.end(), x.begin(), [](unsigned char c) { return (char)std::tolower(c); });
            return x.find("utf-8") != std::string::npos || x.find("utf8") != std::string::npos;
        }
    }
    return false;
}

// Da chiamare una volta, prima di qualunque stampa
inline void Init(double totalSimS, int verbose) {
    State& s = S();
    s.total = totalSimS;
    s.verbose = verbose;
    s.tty = isatty(STDOUT_FILENO);
    const char* term = std::getenv("TERM");
    s.color = s.tty && !std::getenv("NO_COLOR") && !(term && std::string(term) == "dumb");
    s.unicode = s.tty && Utf8Locale();
    s.start = s.lastDraw = std::chrono::steady_clock::now();
}

inline std::string Paint(const char* code, const std::string& x) {
    return S().color ? std::string("\033[") + code + "m" + x + "\033[0m" : x;
}
inline std::string Dash() { return S().unicode ? "\xe2\x80\x93" : "-"; }   // trattino lungo
inline std::string Dot() { return S().unicode ? " \xc2\xb7 " : " | "; }     // separatore " . "
inline std::string Deg() { return S().unicode ? "\xc2\xb0" : " deg"; }      // simbolo dei gradi

inline std::string Fmt(double x, int decimals) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*f", decimals, x);
    return b;
}
// Formattazione predefinita di std::ostream (6 cifre significative): la stessa delle vecchie righe di stdout.txt,
// cosi' gli script ottengono da summary.json esattamente gli stessi numeri
inline std::string Num(double x) { std::ostringstream o; o << x; return o.str(); }

inline std::string Clock(double sec) {
    if (sec < 0) sec = 0;
    long t = (long)(sec + 0.5);
    char b[32];
    if (t >= 3600) std::snprintf(b, sizeof b, "%ld:%02ld:%02ld", t / 3600, (t / 60) % 60, t % 60);
    else std::snprintf(b, sizeof b, "%ld:%02ld", t / 60, t % 60);
    return b;
}

// Elenco di ID compatto: "0-7, 9, 12-19"
inline std::string IdList(std::vector<int> ids) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    std::string out;
    for (size_t i = 0; i < ids.size();) {
        size_t j = i;
        while (j + 1 < ids.size() && ids[j + 1] == ids[j] + 1) j++;
        if (!out.empty()) out += ", ";
        out += std::to_string(ids[i]);
        if (j >= i + 2) out += Dash() + std::to_string(ids[j]);
        else if (j == i + 1) out += ", " + std::to_string(ids[j]);
        i = j + 1;
    }
    return out;
}

inline void ClearProgress() {
    State& s = S();
    if (s.tty && s.drawn) { std::cout << "\r\033[2K"; s.drawn = false; }
}

inline void DrawProgress() {
    State& s = S();
    if (!s.tty || s.total <= 0) return;
    static const char* spinU[] = { "\xe2\xa0\x8b", "\xe2\xa0\x99", "\xe2\xa0\xb9", "\xe2\xa0\xb8", "\xe2\xa0\xbc",
                                   "\xe2\xa0\xb4", "\xe2\xa0\xa6", "\xe2\xa0\xa7", "\xe2\xa0\x87", "\xe2\xa0\x8f" };
    static const char* spinA[] = { "|", "/", "-", "\\" };
    double f = std::min(1.0, std::max(0.0, s.now / s.total));
    const int W = 20;
    int full = (int)(f * W + 1e-9);
    std::string bar;
    for (int i = 0; i < W; i++) bar += i < full ? (s.unicode ? "\xe2\x96\x88" : "#") : (s.unicode ? "\xe2\x96\x91" : "-");
    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - s.start).count();
    std::string eta = f > 0.01 ? Clock(el * (1.0 - f) / f) : "--:--";
    const char* sp = s.unicode ? spinU[s.spin % 10] : spinA[s.spin % 4];
    s.spin++;
    char pct[16];
    std::snprintf(pct, sizeof pct, "%3d%%", (int)(f * 100.0));
    std::string extra;
    if (!s.recentErr.empty()) {
        std::vector<float> v = s.recentErr;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        extra += Dot() + "err " + Fmt(v[v.size() / 2], 1) + " m";
    }
    size_t active = 0;
    for (const auto& kv : s.accusers) active += !kv.second.empty();
    if (!s.recentErr.empty() || active) extra += Dot() + "alarms " + std::to_string(active);
    std::cout << "\r\033[2K  " << Paint("36", sp) << " " << Paint("36", bar) << " " << pct << "  "
              << Fmt(s.now, 0) << "/" << Fmt(s.total, 0) << " s  ETA " << eta << extra << std::flush;
    s.drawn = true;
    s.lastDraw = std::chrono::steady_clock::now();
}

// Una riga di testo sopra la barra di avanzamento
inline void Line(const std::string& text) {
    ClearProgress();
    std::cout << text << "\n";
    if (S().tty) DrawProgress();
}

inline const char* TagColor(const std::string& tag) {
    if (tag == "CAPTURE" || tag == "ALARM" || tag == "EXPEL") return "1;31";
    if (tag == "ATTACK") return "1;33";
    if (tag == "LEAVE" || tag == "JOIN") return "32";
    return "2";
}

inline void EventLine(double t, const std::string& tag, const std::string& text) {
    char b[32];
    std::snprintf(b, sizeof b, "%8.1f s  ", t);
    std::string tg = tag;
    tg.resize(std::max<size_t>(tag.size(), 8), ' ');
    Line(std::string("  ") + b + Paint(TagColor(tag), tg) + " " + text);
}

inline void FlushOne(State::Grp& g) {
    std::string text = g.templ;
    auto put = [&text](const std::string& key, const std::string& val) {
        for (size_t p; (p = text.find(key)) != std::string::npos;) text.replace(p, key.size(), val);
    };
    std::vector<int> ids = g.ids;
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    put("{s}", ids.size() > 1 ? "s" : "");
    put("{n}", std::to_string(ids.size()));
    put("{ids}", IdList(ids));
    EventLine(g.t, g.tag, text);
}

// Stampa i gruppi iniziati prima di `before` (tutti se before < 0), in ordine di tempo
inline void FlushGroups(double before = -1.0) {
    State& s = S();
    std::stable_sort(s.groups.begin(), s.groups.end(), [](const State::Grp& a, const State::Grp& b) { return a.t < b.t; });
    std::vector<State::Grp> keep;
    for (auto& g : s.groups) {
        if (before < 0.0 || g.t < before) FlushOne(g);
        else keep.push_back(g);
    }
    s.groups.swap(keep);
}
inline void FlushGroup() { FlushGroups(); }

// Evento singolo, mostrato se level <= --verbose
inline void Event(double t, const std::string& tag, const std::string& text, int level = 1) {
    if (level > S().verbose) return;
    FlushGroups();
    EventLine(t, tag, text);
}

// Evento raggruppabile: stesso tag e stesso testo entro 0,5 s simulati -> una sola riga con tutti gli ID.
// Nel testo: {ids} = elenco compatto, {n} = quanti, {s} = "s" se piu' di uno
inline void Group(double t, const std::string& tag, const std::string& templ, int id, int level = 1) {
    State& s = S();
    if (level > s.verbose) return;
    FlushGroups(t - 0.5);
    for (auto& g : s.groups)
        if (g.tag == tag && g.templ == templ) { g.ids.push_back(id); return; }
    s.groups.push_back({ t, tag, templ, { id } });
}

// Testo libero (dettagli e debug), mostrato se level <= --verbose
inline void Raw(const std::string& text, int level = 2) {
    if (level > S().verbose) return;
    FlushGroups();
    Line(text);
}

// Avanzamento: chiamato dal simulatore a intervalli regolari di tempo simulato (eventi gia' esistenti)
inline void Tick(double now) {
    State& s = S();
    s.now = now;
    FlushGroups(now - 0.5);
    if (s.tty && std::chrono::duration<double>(std::chrono::steady_clock::now() - s.lastDraw).count() >= 0.1)
        DrawProgress();
}
inline void Json(const std::string& key, const std::string& value);
inline std::string JsonStr(const std::string& x);

// ---------------------------------------------------------------- verita' della simulazione (solo per le stampe)
inline void SetScenario(const std::vector<int>& insiders, double attackS, double spoofS) {
    State& s = S();
    s.insiders = std::set<int>(insiders.begin(), insiders.end());
    s.attackS = insiders.empty() ? -1.0 : attackS;
    s.spoofS = spoofS;
}
inline bool IsLiar(int id, double t) { const State& s = S(); return s.insiders.count(id) && s.attackS >= 0 && t >= s.attackS; }
inline double FirstAttack() {
    const State& s = S();
    double a = -1.0;
    for (double x : { s.attackS, s.spoofS }) if (x >= 0 && (a < 0 || x < a)) a = x;
    return a;
}

// Un osservatore inizia o smette di vedere un allarme collettivo su `target`. Episodio = dal primo osservatore che accusa
// all'ultimo che smette. Evento all'inizio (interno scoperto / falso allarme / transitorio dello spoofer) e alla fine.
inline void AlarmObserver(double t, int target, int observer, bool on) {
    State& s = S();
    auto& acc = s.accusers[target];
    bool wasOn = !acc.empty();
    if (on) acc.insert(observer); else acc.erase(observer);
    bool isOn = !acc.empty();
    if (isOn && !wasOn) {
        bool liar = IsLiar(target, t);
        bool onset = !liar && s.spoofS >= 0 && t >= s.spoofS && s.firstConsensus < 0;
        s.openEp[target] = s.episodes.size();
        s.episodes.push_back({ target, t, -1.0, liar, onset });
        if (liar && !s.detected.count(target)) {
            s.detected[target] = t;
            Group(t, "ALARM", "insider{s} {ids} detected " + Fmt(t - s.attackS, 1) + " s after the attack", target, 1);
        } else if (liar) {
            Group(t, "ALARM", "alarm on insider{s} {ids} again", target, 1);
        } else if (onset) {
            Group(t, "ALARM", "spoofer onset: alarm on drone{s} {ids}", target, 1);
        } else {
            Group(t, "ALARM", "false alarm on honest drone{s} {ids}", target, 1);
        }
    } else if (!isOn && wasOn) {
        auto it = s.openEp.find(target);
        if (it != s.openEp.end()) {
            State::Ep& e = s.episodes[it->second];
            e.end = t;
            s.openEp.erase(it);
            if (!e.onset)   // il transitorio dello spoofer si chiude con il consenso: non serve una riga per drone
                Group(t, "CLEAR", "alarm on drone{s} {ids} cleared after " + Fmt(t - e.start, 1) + " s", target, 1);
        }
    }
}

inline void CaptureConsensus(double t) { if (S().firstConsensus < 0) S().firstConsensus = t; }

// Una riga del CSV: errore di stima di `target` visto da `observer` (solo lettura, per barra e riepilogo)
inline void Observe(double t, int target, int observer, double err) {
    State& s = S();
    if (IsLiar(target, t) || IsLiar(observer, t)) return;
    const size_t K = 400;
    if (s.recentErr.size() < K) s.recentErr.push_back((float)err);
    else { s.recentErr[s.recentPos] = (float)err; s.recentPos = (s.recentPos + 1) % K; }
    double a = FirstAttack();
    if (t >= 15.0 && (a < 0 || t < a)) s.preErr.push_back((float)err);   // 15 s: come il test dei sospetti
}

// Riepilogo dei risultati (in testa al riepilogo) e chiavi in summary.json
inline void ScenarioSummary(uint32_t nConsensus, uint32_t nDrones, const std::vector<std::string>& files) {
    State& s = S();
    std::vector<std::pair<std::string, std::string>> top;
    double end = s.total;
    if (!s.insiders.empty()) {
        std::string det, miss, js;
        for (int id : s.insiders) {
            auto it = s.detected.find(id);
            if (it == s.detected.end()) { miss += (miss.empty() ? "" : ", ") + std::to_string(id); continue; }
            det += (det.empty() ? "drone " : ", ") + std::to_string(id) + " after " + Fmt(it->second - s.attackS, 1) + " s";
            js += std::string(js.empty() ? "" : ", ") + JsonStr(std::to_string(id)) + ": " + Fmt(it->second - s.attackS, 2);
        }
        top.emplace_back("insiders", std::to_string(s.detected.size()) + "/" + std::to_string(s.insiders.size())
                         + " alarmed" + (det.empty() ? "" : ": " + det) + (miss.empty() ? "" : "; never alarmed: " + miss));
        Json("insider_first_alarm_s", "{" + js + "}");
    }
    std::vector<int> fa, onset;
    double faTime = 0.0;
    size_t faEp = 0;
    for (const auto& e : s.episodes) {
        if (e.insider) continue;
        if (e.onset) { onset.push_back(e.target); continue; }
        fa.push_back(e.target); faEp++;
        faTime += (e.end >= 0 ? e.end : end) - e.start;
    }
    std::sort(fa.begin(), fa.end()); fa.erase(std::unique(fa.begin(), fa.end()), fa.end());
    if (!s.insiders.empty() || !fa.empty())
        top.emplace_back("false alarms", fa.empty() ? "none" : "honest drone" + std::string(fa.size() > 1 ? "s " : " ")
                         + IdList(fa) + " (" + std::to_string(faEp) + " episode" + (faEp > 1 ? "s, " : ", ")
                         + Fmt(faTime, 1) + " s in total)");
    std::string faJs;
    for (int id : fa) faJs += (faJs.empty() ? "" : ", ") + std::to_string(id);
    Json("false_alarm_drones", "[" + faJs + "]");
    std::sort(onset.begin(), onset.end()); onset.erase(std::unique(onset.begin(), onset.end()), onset.end());
    if (!onset.empty()) top.emplace_back("spoofer onset", "drone" + std::string(onset.size() > 1 ? "s " : " ") + IdList(onset)
                                         + " alarmed before any capture consensus");
    if (s.spoofS >= 0)
        top.emplace_back("capture", nConsensus == 0 ? "no capture consensus" :
                         std::to_string(nConsensus) + "/" + std::to_string(nDrones) + " drones, first consensus "
                         + Fmt(s.firstConsensus - s.spoofS, 2) + " s after the spoofer");
    if (!s.preErr.empty()) {
        std::vector<float> v = s.preErr;
        std::sort(v.begin(), v.end());
        auto q = [&v](double p) { return v[std::min(v.size() - 1, (size_t)(p * (v.size() - 1) + 0.5))]; };
        top.emplace_back("localization", "honest drones " + std::string(FirstAttack() >= 0 ? "before the attack" : "after 15 s")
                         + ": median " + Fmt(q(0.5), 2) + " m, p95 " + Fmt(q(0.95), 2) + " m");
    }
    std::string fl;
    for (const auto& f : files) fl += (fl.empty() ? "" : ", ") + f;
    top.emplace_back("files", fl);
    top.insert(top.end(), s.summary.begin(), s.summary.end());
    s.summary.swap(top);
}

// ---------------------------------------------------------------- intestazione, riepilogo, summary.json
inline void Title(const std::string& t) { S().title = t; }
inline void Config(const std::string& key, const std::string& value) { S().config.emplace_back(key, value); }
inline void Summary(const std::string& key, const std::string& value) { S().summary.emplace_back(key, value); }
// value e' gia' JSON valido (numero, stringa tra virgolette, lista o oggetto)
inline void Json(const std::string& key, const std::string& value) { S().json.emplace_back(key, value); }
inline std::string JsonStr(const std::string& x) {
    std::string o = "\"";
    for (char c : x) { if (c == '"' || c == '\\') o += '\\'; o += c; }
    return o + "\"";
}

inline void PrintBlock(const std::vector<std::pair<std::string, std::string>>& rows) {
    for (const auto& [k, v] : rows) {
        std::string key = k;
        key.resize(std::max<size_t>(k.size(), 13), ' ');
        // valori lunghi a capo, allineati sotto la colonna dei valori (righe entro 80 colonne)
        const size_t col = 2 + key.size() + 2, width = 80 > col + 20 ? 80 - col : 20;
        std::string rest = v, out;
        while (rest.size() > width) {
            size_t cut = rest.rfind(", ", width - 1);   // meglio dopo una virgola: non separa un numero dalla sua unita'
            if (cut != std::string::npos && cut > width / 2) cut += 1;
            else cut = rest.rfind(' ', width);
            if (cut == std::string::npos || cut == 0) break;
            out += rest.substr(0, cut) + "\n" + std::string(col, ' ');
            rest = rest.substr(cut + 1);
        }
        std::cout << "  " << Paint(k == "warning" ? "1;33" : "1", key) << "  " << out << rest << "\n";
    }
}

inline void PrintHeader() {
    State& s = S();
    std::cout << Paint("1", s.title) << "\n";
    PrintBlock(s.config);
    std::cout << std::endl;
    if (s.tty) DrawProgress();
}

// Fine: chiude l'avanzamento, stampa riepilogo e scrive summary.json nella cartella corrente
inline void Finish(const std::string& jsonPath = "summary.json") {
    State& s = S();
    FlushGroup();
    ClearProgress();
    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - s.start).count();
    std::string speed = el > 0 ? " (" + Fmt(s.total / el, 1) + (s.unicode ? "\xc3\x97" : "x") + " real time)" : "";
    std::cout << "  " << Paint("1;32", s.unicode ? "\xe2\x9c\x93 done" : "done") << "  " << Fmt(s.total, 0) << "/"
              << Fmt(s.total, 0) << " s in " << Clock(el) << speed << "\n\n";
    if (!s.summary.empty()) PrintBlock(s.summary);
    std::cout << std::flush;
    std::ofstream js(jsonPath);
    js << "{\n";
    for (size_t i = 0; i < s.json.size(); i++)
        js << "  " << JsonStr(s.json[i].first) << ": " << s.json[i].second << (i + 1 < s.json.size() ? "," : "") << "\n";
    js << "}\n";
}

}  // namespace ui
