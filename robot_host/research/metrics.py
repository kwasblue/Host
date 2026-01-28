# robot_host/research/metrics.py
from __future__ import annotations
import json
from typing import Any

def load_jsonl(path: str) -> list[dict[str, Any]]:
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            rows.append(json.loads(line))
    return rows

def basic_metrics(jsonl_path: str) -> dict[str, Any]:
    rows = load_jsonl(jsonl_path)

    rx = [r for r in rows if r.get("event") == "transport.rx"]
    tx = [r for r in rows if r.get("event") == "transport.tx"]

    return {
        "counts": {"rx": len(rx), "tx": len(tx), "total": len(rows)},
        "bytes": {
            "rx_total": sum(int(r.get("n", 0)) for r in rx),
            "tx_total": sum(int(r.get("n", 0)) for r in tx),
        },
    }
