<p align="center">
  <img src="assets/banner.svg" alt="robotics-lab" width="100%">
</p>

<p align="center"><em>ROS 2 and CAN bus tools, plus re-implementations of robotics papers.</em></p>

<p align="center">
  <img alt="Status: in development" src="https://img.shields.io/badge/status-in%20development-8a3a24?style=flat-square">
  <img alt="License: MIT" src="https://img.shields.io/badge/license-MIT-1c1a17?style=flat-square">
  <img alt="Python 3.11+" src="https://img.shields.io/badge/python-3.11%2B-1c1a17?style=flat-square">
  <img alt="ROS 2 and CAN" src="https://img.shields.io/badge/ROS%202%20%7C%20CAN-1c1a17?style=flat-square">
</p>

---

## Features

- **ROS 2 node and latency tooling**: measure topic rates, end-to-end latency and jitter across a node graph.
- **CAN bus logging and decoding**: record from SocketCAN or a virtual bus, decode with a DBC file, export to CSV or Parquet.
- **Paper re-implementations in simulation**: compact, readable re-implementations of selected robotics papers (to be announced).

## Architecture

```mermaid
flowchart LR
  subgraph ROS["ROS 2 tooling"]
    direction TB
    N1["Node graph"]:::ink --> LT["Latency + jitter probes"]:::ink
    LT --> RP["Report<br/>HTML · CSV"]:::ink
  end
  subgraph CAN["CAN bus utilities"]
    direction TB
    B["SocketCAN / virtual bus"]:::ink --> LG["Logger"]:::ink
    LG --> DC["DBC decoder"]:::ink
    DC --> EXP["CSV · Parquet · plots"]:::ink
  end
  subgraph PAP["Paper re-implementations"]
    direction TB
    P1["Paper 1 (to be announced)"]:::rust --> SIM["Simulation"]:::ink
    P2["Paper 2 (to be announced)"]:::rust --> SIM
  end

  classDef ink fill:#f3efe6,stroke:#1c1a17,color:#1c1a17,stroke-width:1px;
  classDef rust fill:#f3efe6,stroke:#8a3a24,color:#8a3a24,stroke-width:1.5px;
```

## Quickstart

> **Coming soon.** The commands below show the planned interface.

```bash
git clone https://github.com/ModelShifts/robotics-lab
cd robotics-lab
cp .env.example .env   # choose a backend
pip install -e .
robotics-lab ros latency --topic /joint_states
robotics-lab can log --channel vcan0 --dbc robot.dbc --out log.parquet
```

## Try it

> **Coming soon.** A hosted demo will be linked here and on [modelshifts.com/work](https://modelshifts.com/work).

## Roadmap

- [x] Repository scaffold and README
- [ ] ROS 2 topic rate and latency probes
- [ ] CAN logger for SocketCAN and virtual buses
- [ ] DBC decoding and export
- [ ] First paper re-implementation in simulation
- [ ] Second paper re-implementation in simulation

## License

[MIT](LICENSE) © Modelshifts Technology Pte. Ltd.

<p align="center"><sub>Part of the <a href="https://modelshifts.com/work">Modelshifts</a> showcase.</sub></p>
