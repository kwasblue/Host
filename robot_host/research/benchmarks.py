# robot_host/research/benchmarks.py
from __future__ import annotations
import json
from robot_host.research.metrics import basic_metrics

def run_benchmark(session_jsonl: str, out_json: str | None = None):
    m = basic_metrics(session_jsonl)
    if out_json:
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump(m, f, indent=2)
    return m
