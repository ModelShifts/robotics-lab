# Try robotics-lab v1

Python 3.11 or newer. No ROS 2 install, no CAN hardware, and no model download.

```bash
git clone https://github.com/ModelShifts/robotics-lab
cd robotics-lab
git checkout v1
python -m venv .venv && source .venv/bin/activate
pip install -e ".[demo]"
cp .env.example .env
```

Bag analysis prints topic rate, jitter, and end-to-end latency for the synthetic joint loop:

```bash
robotics-lab ros analyze samples/ros/node_graph
```

The committed CAN trace is the same schedule, decoded with the DBC in the repo:

```bash
robotics-lab can analyze samples/can/trace.csv --dbc samples/can/robot.dbc
```

A live virtual-bus capture (the logger also sends the synthetic frames, because a virtual channel is otherwise empty):

```bash
robotics-lab can log --interface virtual --channel robotlab --dbc samples/can/robot.dbc --seconds 1 --out out/can
```

That writes `trace.csv`, `decoded.csv`, `decoded.parquet`, and `report.json`.

Paper figures:

```bash
robotics-lab papers dmp --out out/dmp
robotics-lab papers ilqr --out out/ilqr
```

Web demo at http://127.0.0.1:7860 (three tabs: the bag, the CAN trace, the two papers):

```bash
robotics-lab serve
```

Checks against the generator settings, not against external data:

```bash
pip install -e ".[dev]"
pytest
python scripts/evaluate.py
python scripts/make_samples.py && git diff --exit-code -- samples/
```

`evaluate.py` exits 0 when every check passes. There is no language-model backend to configure.
