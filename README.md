# Decentralized Localization of a GNSS Spoofer from Inside a Captured Drone Swarm

> **A drone swarm that notices it has been captured by a GNSS spoofer, keeps flying on its own radios, and points at the spoofer's antenna.**
> No ground infrastructure, no coordinator, no extra messages: only the swarm's UWB radios and GNSS receivers, inside one TDMA protocol.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/20)
[![ns-3](https://img.shields.io/badge/ns--3-3.44-blue.svg)](https://www.nsnam.org/)
[![Eigen](https://img.shields.io/badge/Eigen-3.3%2B-brightgreen.svg)](https://eigen.tuxfamily.org/)
[![Python 3](https://img.shields.io/badge/python-3-3776AB.svg?logo=python&logoColor=white)](https://www.python.org/)
[![Platform: Linux](https://img.shields.io/badge/platform-Linux-lightgrey.svg?logo=linux&logoColor=white)](#requirements)
[![Status: research prototype](https://img.shields.io/badge/status-research%20prototype-orange.svg)](#limitations)
[![Last commit](https://img.shields.io/github/last-commit/jacopodilauro/decentralized-multilateralization-drone-swarm/feature/realistic-simulator)](https://github.com/jacopodilauro/decentralized-multilateralization-drone-swarm/commits/feature/realistic-simulator)

---

This repository contains an [ns-3](https://www.nsnam.org/) simulator of a drone swarm that defends itself against GNSS spoofing. A single-antenna spoofer can silently move a whole swarm to a false position, but it leaves two fingerprints that the swarm itself can measure:

- **All receivers report the same position, while the real distances between drones do not change.** The swarm measures those distances with ultra-wideband (UWB) ranging, so the mismatch exposes the capture.
- **Every captured receiver emits its pulse-per-second (PPS) late by its distance from the spoofer antenna.** The differences between PPS edges are a time-difference-of-arrival measurement of the spoofer.

On top of these, each drone localizes its neighbors with a cooperative filter and the swarm exposes members that lie about their own position (*insiders*) by quorum voting.

## Contents

- [Results](#results)
- [How it works](#how-it-works)
- [Requirements](#requirements)
- [Build and run](#build-and-run)
- [What you see](#what-you-see)
- [Main options](#main-options)
- [Output files](#output-files)
- [Reproducing the evaluation](#reproducing-the-evaluation)
- [Repository layout](#repository-layout)
- [Tests](#tests)
- [Limitations](#limitations)
- [License](#license)

## Results

Evaluation over 1,190 simulations with 5 to 50 drones, 12 m apart, 10 seeds per configuration (5 with 50 drones). All numbers come from simulation.

| What | Result |
|:---|:---|
| Capture recognized | by 100% of the drones, in about ten TDMA frames (1.05 s with 20 drones, 2.65 s with 50) |
| Spoofer bearing, 8 or more drones | 1–4.3 m of cross-range error with the antenna within 100 m; 14–30 m at 1 km |
| Spoofer range, 50 drones | resolved in 90% of the runs at 200 m, 70% at 500 m, 46% at 1 km |
| Spoofer range, 20 drones | resolved in 96% of the runs at 100 m, 76% at 200 m, 16% at 1 km |
| One insider | exposed in 2–3 s with up to 20 drones (4.5 s with 50) |
| Insider tolerance | at least `k + 5` drones are needed to resist `k` insiders; 12–30 drones hold up to 42–47% of insiders, 50 drones up to 24% |

A larger swarm resolves the spoofer range farther away, because the range is read from the curvature of the wavefront across the swarm.

## How it works

Every drone runs the same pipeline, and every decision is taken by a quorum vote carried in the TDMA broadcasts:

1. **Ranging.** Three consecutive broadcasts between two drones form a double-sided two-way ranging (DS-TWR) exchange, so ranging needs neither synchronized clocks nor extra messages.
2. **Cooperative localization.** One extended Kalman filter per neighbor fuses the direct distance with distances shared by other drones, compensating their age.
3. **Insider detection.** A chi-square test compares the UWB estimate of a neighbor with the GNSS position it declares; votes are shared and a two-thirds quorum raises a collective alarm.
4. **Capture detection.** Each drone compares GNSS-implied and UWB-measured distances between pairs of other drones. Once two thirds of the swarm agree, drones stop trusting GNSS and keep the formation with UWB and on-board velocity sensors.
5. **Spoofer localization.** Captured drones timestamp their PPS with the UWB clock; PPS differences are logged and solved offline for the bearing and range of the antenna (`tools/spoofer_locate.py`).

## Requirements

- Linux, a C++20 compiler and CMake
- [ns-3](https://www.nsnam.org/) 3.44
- [Eigen](https://eigen.tuxfamily.org/) 3.3 or newer
- Python 3 with `numpy`, `pandas` and `matplotlib`, only for the analysis scripts in `tools/`

## Build and run

The simulator builds as an ns-3 *scratch* program. Link the repository into your ns-3 tree and build:

```bash
git clone -b feature/realistic-simulator \
    https://github.com/jacopodilauro/decentralized-multilateralization-drone-swarm.git
ln -s "$PWD/decentralized-multilateralization-drone-swarm" ~/ns-3-dev/scratch/pleion
cd ~/ns-3-dev && ./ns3 build
```

Run it from an empty folder, because the output files are written to the current directory:

```bash
mkdir -p /tmp/run && cd /tmp/run
~/ns-3-dev/build/distry_mlat26v4-nomanager --netanim=0 --setnGuests=0 --formation=shell \
    --radio=uwb --uwbError=real --setnDrones=20 --anchorVel=gps \
    --spoof=1 --spoofTime=40 --spoofDist=100 --setSimTime=90
```

`--PrintHelp` lists every option with its default.

Without a full ns-3 tree, the simulator can also be built against the ns-3 Python package (`pip install ns3`):

```bash
NS3_PREFIX=$(python3 -c "import importlib.util as u; print(u.find_spec('ns3').submodule_search_locations[0])") \
    tools/build_standalone.sh ./sim
```

## What you see

The command above simulates 20 drones captured by a spoofer 100 m away. On a terminal, a progress line stays at the bottom while events scroll above it:

```text
Distry MLAT-26 · seed 1 · 90 s simulated
  swarm          20 drones (20 base, 0 guests), shell formation
  geometry       radius 16.0 m, spacing 12 m, rotation 1 m/s
  TDMA           frame 100 ms (20 slots of 5 ms)
  radio          UWB, 50% reception at 85.4 m (sensitivity -100 dBm)
  ranging        DS-TWR from TDMA broadcasts
  UWB error      realistic, P_LOS = exp(-d/500 m), NLOS correlated over 10 m
  GNSS           realistic receiver, chi2 threshold 16.27
  spoofer        100 m away, azimuth -37°, from t = 40 s

      40.0 s  ATTACK   spoofer on, 100 m away, azimuth -37°
      41.0 s  ALARM    spoofer onset: alarm on drones 0–19
      41.0 s  CAPTURE  swarm captured: consensus at drones 0–19
  ✓ done  90/90 s in 0:31 (2.9× real time)

  spoofer onset  drones 0–19 briefly alarmed around the capture consensus
  capture        20/20 drones, first consensus 1.03 s after the spoofer
  localization   honest drones before the attack: median 5.40 m, p95 6.40 m
  files          tdma_security_log.csv, ground_truth.csv, gnss_log.csv,
                 pps_log.csv, summary.json
  UWB radio      99.98% of 342000 packets received
  by range       0-30 m 99.99% · 30-60 m 99.94%
  packets        largest 304 B
  PPS            18240 differences, 0 rejected (fit residual > 10 m)
  UWB errors     341936 messages, 4.04% NLOS, 139 episodes (10 undetected,
                 mean 5.0 s)
  NLOS flag      raised on 5.02% of LOS and 95.09% of NLOS messages
```

Each line is one event with its simulated time: `JOIN` and `LEAVE` for guests entering and leaving, `ATTACK`, `ALARM` (with whether the accused drone is an insider or an honest one), `CLEAR`, `CAPTURE` and `EXPEL`. The figures in the final summary are live estimates with simple definitions; the metrics of the evaluation come from the analysis scripts. When the output is redirected to a file, the progress line and the colors are omitted. `--verbose=0` prints only the configuration and the summary, `--verbose=2` adds per-drone details.

## Main options

| Option | Default | Meaning |
|:---|:---|:---|
| `--setnDrones`, `--setnGuests` | 8, 12 | base drones and guest drones |
| `--formation`, `--spacing` | `legacy`, 12 m | `shell`: spherical shell with constant spacing |
| `--radio` | `wifi` | `uwb`: packet loss that grows with distance |
| `--uwbError` | `legacy` | `real`: correlated NLOS episodes and distance-dependent noise |
| `--anchorVel` | `ekf` | `gps`: propagate anchors with their declared GNSS velocity; **required with 30 or more drones**, where `ekf` diverges |
| `--targetsId`, `--attackTime` | 0, 200 s | insiders (comma-separated IDs) and start of their attack |
| `--insiderSpread` | false | insiders drift each in its own direction instead of all north |
| `--spoof`, `--spoofDist`, `--spoofAz`, `--spoofTime` | false, 500 m, −37°, attack time | external single-antenna spoofer |
| `--mixed` | false | keep the insiders together with the external spoofer |
| `--runId`, `--setSimTime` | 1, 240 s | seed and simulated duration |
| `--verbose`, `--netanim` | 1, 1 | detail of the terminal output; NetAnim trace (set `--netanim=0` for experiments) |

## Output files

| File | Content |
|:---|:---|
| `tdma_security_log.csv` | one row per observation: estimated, declared and true position of the observed drone, votes, alarms, test statistics |
| `ground_truth.csv` | true position of every drone every 0.5 s |
| `gnss_log.csv`, `pps_log.csv` | with `--spoof=1`: receiver states and PPS differences between drones |
| `summary.json` | values of the run read by the analysis scripts (packet size, UWB reception, capture consensus, spoofer antenna, ...) |

## Reproducing the evaluation

```bash
export SIM=~/ns-3-dev/build/distry_mlat26v4-nomanager
python3 tools/paper_campaign.py --profilo prova --jobs 4 --out runs/check      # 7 short runs, about a minute
python3 tools/paper_campaign.py --profilo completo --jobs 6 --out runs/paper   # 1,190 runs
python3 tools/paper_report.py   --out runs/paper                               # tables and report
python3 tools/paper_figures.py  --report runs/paper/report --out runs/paper/figures
```

The full campaign took about 15 hours on an 8-thread laptop with 6 parallel jobs. It can be interrupted and resumed with the same command: finished runs are skipped. The comments, the help texts and the reports of the scripts are in Italian.

## Repository layout

```text
include/    headers: application, EKF, DS-TWR, GNSS receiver and spoofer, clocks, UWB link and error models,
            on-board sensors, packet header, terminal output
src/        sources: main.cpp (scenario and options), UwbSecurityApp.cpp (protocol, filters, votes, capture),
            UwbHeader.cpp, UWBChannel.cpp, EKF.cpp, Trajectories.cpp
tests/      isolated tests of the single components
tools/      campaign, report, figures, spoofer localization, regression helpers
reference/  reference metrics of earlier development steps, used by the regression checks
viwer.py    interactive viewer of a single run
plotv2.py   viewer v2: English labels, button for the votes against every drone
```

## Tests

```bash
tools/run_tests.sh                                    # EKF, DS-TWR, GNSS receiver, UWB link
NS3_PREFIX=/path/to/ns3 tools/run_tests.sh            # also packet header, UWB channel, ns-3 propagation loss
```

Every change to the simulator is checked against the previous version on the same seeds: options are introduced with the previous behavior as default, and the output files must stay byte-for-byte identical.

## Limitations

- **Simulation only.** Sensor parameters follow public specifications or are documented assumptions; they have not been calibrated on hardware.
- **Transport.** Packets travel on the ns-3 Wi-Fi stack, and a UWB link model decides whether each one is received; packet loss does not depend on packet length.
- **Attacker.** The external spoofer is static and has a single antenna; insiders lie about their position but do not vote strategically, and votes are not authenticated.
- **PPS assumption.** The spoofer localization assumes that the receiver's PPS follows the counterfeit time immediately after capture. `--ppsTauS` and `--ppsBiasNs` model a slower transition and per-receiver delays, to study the sensitivity to this assumption.

## License

Released under the [MIT License](LICENSE).
