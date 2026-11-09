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
    const int W = 24;
    int full = (int)(f * W + 1e-9);
    std::string bar;
    for (int i = 0; i < W; i++) bar += i < full ? (s.unicode ? "\xe2\x96\x88" : "#") : (s.unicode ? "\xe2\x96\x91" : "-");
    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - s.start).count();
    std::string eta = f > 0.01 ? Clock(el * (1.0 - f) / f) : "--:--";
    const char* sp = s.unicode ? spinU[s.spin % 10] : spinA[s.spin % 4];
    s.spin++;
    char pct[16];
    std::snprintf(pct, sizeof pct, "%3d%%", (int)(f * 100.0));
    std::cout << "\r\033[2K  " << Paint("36", sp) << " " << Paint("36", bar) << " " << pct << "  "
              << Fmt(s.now, 0) << "/" << Fmt(s.total, 0) << " s  ETA " << eta << std::flush;
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
        key.resize(std::max<size_t>(k.size(), 10), ' ');
        std::cout << "  " << Paint("1", key) << "  " << v << "\n";
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
