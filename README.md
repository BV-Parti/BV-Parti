# BV-Parti (Anonymous Submission)

This repository accompanies a CAV 2026 regular paper submission. It is **anonymized for double-blind review**. Please do not attempt to infer or disclose author identities.

## Overview

BV-Parti is a distributed and parallel SMT(BV) solving framework that strengthens the traditional preprocess-and-bit-blast pipeline with **semantic preprocessing**. It computes a *feasible-domain* snapshot over the global bit-vector DAG, extracts high-value consequences (e.g., fixed bits and wrap-aware bounds), and injects them back as ordinary BV constraints so they persist across rewriting, bit-blasting, and distributed search. These feasible domains are also used to **guide partitioning**, selecting split points aligned with semantic boundaries (e.g., modular wrap points) rather than purely syntactic bit slicing.

The system runs in **parallel (single-node)** and **distributed (multi-node)** modes using an MPI-based leader-coordinator-worker architecture. Each partition is solved by a standard SMT solver via SMT-LIB v2.

## Artifact Contents

- `src/` : Core runtime (leader/coordinator/dispatcher), launcher, and utilities.
- `src/partitioner/` : Bit-vector partitioner and feasible-domain propagation (C++/Meson).
- `script/build.py` : Build & staging script (creates `bin/`).
- `test/instances/` : Small BV benchmarks.
- `test/configs/` : Example JSON configs for parallel/distributed runs.
- `linux-pre_built/` : Prebuilt solver binaries (Linux) used by the launcher.
- `experiment_results/` : Raw CSV results and summary tables derived from them.

## Requirements

- Linux (tested on Ubuntu 20.04/22.04)
- Python 3.7+
- C++17 compiler toolchain
- Meson + Ninja
- MPI runtime (e.g., OpenMPI) and `mpi4py`

Example (Ubuntu):

```bash
sudo apt-get install -y build-essential meson ninja-build openmpi-bin libopenmpi-dev python3-mpi4py
```

## Build

Run from the repository root:

```bash
python3 script/build.py --build-type release
```

This will:

- Stage Python entry points into `bin/`
- Build the partitioner under `src/partitioner/`
- Copy required binaries into `bin/binaries/`

## Base Solver Setup

The launcher expects solver binaries under `bin/binaries/`. The build script stages a tested BV solver (`bitwuzla-0.8.0-bin`) there by default. To use a different SMT(BV) solver, place its executable in `bin/binaries/` and update the `base_solver` field in your config JSON.

## Quick Start (Parallel)

1) Edit `test/configs/parallel-64.json` to match your machine (paths and core count).
2) Run:

```bash
python3 bin/BV-Parti_launcher.py test/configs/parallel-64.json
```

Outputs are written to the configured `output_dir`:

- `logs/` : runtime logs
- `rankfile` : MPI rankfile
- solver outputs and intermediate data

## Distributed Runs (Multi-node)

**Important:** All nodes must see the *same absolute paths* to the formula and output directories (e.g., shared filesystem).

1) Edit `test/configs/distributed-128.json`:
   - `network_subnet` should match your cluster NIC subnet (CIDR).
   - `worker_node_ips` and `worker_node_cores` must match your nodes.
2) Run from the head node:

```bash
python3 bin/BV-Parti_launcher.py test/configs/distributed-128.json
```

The launcher passes `network_subnet` into Open MPI to restrict TCP traffic to the desired interface.

## Configuration Fields

| Field | Description | Required | Notes |
| --- | --- | --- | --- |
| `formula_file` | SMT-LIB v2 input file | Yes | Use absolute path for distributed runs |
| `output_dir` | Output directory | Yes | Created if missing |
| `timeout_seconds` | Global timeout (seconds) | Yes | 0 disables timeout |
| `base_solver` | Solver binary name in `bin/binaries/` | Yes | Must accept SMT-LIB v2 |
| `mode` | `parallel` or `distributed` | Yes | Controls runtime mode |
| `parallel_core` | Core count | Parallel | Suggested >= 8 |
| `network_subnet` | IPv4 CIDR for MPI TCP | Distributed | Example: `192.168.1.0/24` |
| `worker_node_ips` | IP list for workers | Distributed | Order matches core list |
| `worker_node_cores` | Core counts for workers | Distributed | Same length as IP list |
| `max_worker_memory_mb` | Per-worker memory cap | Optional | 0 = unlimited |
| `output_total_time` | Print wall-clock time | Optional | Default `false` |

## Notes on Parallel Scheduling

For small `parallel_core` (<= 8), the launcher uses a simplified mode with a single isolated coordinator. For larger runs, it reserves a few cores on the first node for a dedicated coordinator and leader.

## Experiment Results

The CSV files under `experiment_results/raw-data/` contain per-instance results. The summaries below are computed directly from those CSVs.

Setup summary:

- Dataset size: 46,191 instances (rows in each CSV).
- Timeout: 1200 seconds.
- "Unsolved" includes any non-`sat`/`unsat` result (e.g., `timeout`, `unknown`, `memoryout`, `error`, `unsolved`).
- PAR-2 uses 2 * timeout for unsolved instances.

Single-thread baselines:

| Solver | Solved | Unsolved | SAT | UNSAT | PAR-2 (s) |
| --- | --- | --- | --- | --- | --- |
| bitwuzla | 45728 | 463 | 18087 | 27641 | 30.49 |
| bitwuzla-fdp | 45779 | 412 | 18110 | 27669 | 27.13 |
| cvc5 | 44799 | 1392 | 17921 | 26878 | 83.95 |
| z3 | 43670 | 2521 | 17232 | 26438 | 146.87 |
| yices2 | 44819 | 1372 | 17927 | 26892 | 76.98 |
| stp | 45512 | 679 | 17925 | 27587 | 44.77 |

BV-Parti scaling (parallel cores):

| Cores | Solved | Unsolved | SAT | UNSAT | PAR-2 (s) |
| --- | --- | --- | --- | --- | --- |
| 4 | 45841 | 350 | 18140 | 27701 | 22.58 |
| 8 | 45851 | 340 | 18144 | 27707 | 21.75 |
| 16 | 45866 | 325 | 18152 | 27714 | 20.86 |
| 32 | 45878 | 313 | 18158 | 27720 | 19.97 |
| 64 | 45901 | 290 | 18164 | 27737 | 18.83 |
| 128 | 45922 | 269 | 18172 | 27750 | 17.39 |

Parallel baselines (selected core counts):

| Solver | Cores | Solved | Unsolved | PAR-2 (s) |
| --- | --- | --- | --- | --- |
| bitwuzla | 64 | 45779 | 412 | 24.77 |
| bitwuzla | 128 | 45766 | 425 | 25.75 |
| cvc5 | 64 | 43804 | 2387 | 136.96 |
| cvc5 | 128 | 43726 | 2465 | 140.50 |
| pboolector | 64 | 43943 | 2248 | 129.89 |
| pboolector | 128 | 43760 | 2431 | 139.10 |

## Directory Structure (Top Level)

```
BV-Parti/
|- bin/                        # Staged entry points + binaries (created by build)
|- experiment_results/         # Experiment data (to be added)
|- linux-pre_built/            # Prebuilt solver binaries (Linux)
|- script/                     # Build and helper scripts
|- src/                        # Core implementation
|- test/                       # Example configs + benchmark instances
`- README.md
```

## Anonymity

All identifying metadata has been removed or generalized for double-blind review. If you notice remaining identifiers, please report them so we can correct the anonymization.
