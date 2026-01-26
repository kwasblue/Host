import pytest
from fakes.fake_async_transport import FakeAsyncTransport
from helpers import CapturingBus
import os
import json
import time
from pathlib import Path
import asyncio

@pytest.fixture
def bus():
    return CapturingBus()


@pytest.fixture
def transport():
    return FakeAsyncTransport(auto_ack=True)

def pytest_addoption(parser):
    parser.addoption("--mcu-port", action="store", default=os.getenv("MCU_PORT", ""))

def pytest_configure(config):
    config.addinivalue_line("markers", "hil: hardware-in-the-loop tests (requires MCU connected)")
def attach_bus_dump(client, path: str, cycle: int | None = None):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    f = open(path, "a", buffering=1)

    def _ser(x):
        try:
            return json.dumps(x, default=str, ensure_ascii=False)
        except Exception:
            return repr(x)

    def log(topic, obj):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        cyc = f"cycle={cycle} " if cycle is not None else ""
        f.write(f"{ts} {cyc}{topic} {_ser(obj)}\n")

    sub = getattr(client.bus, "subscribe", None)
    if not callable(sub):
        return lambda: f.close()

    # If your bus supports wildcard, use it. If not, list topics you care about.
    wildcard = getattr(client.bus, "subscribe_all", None)
    if callable(wildcard):
        wildcard(lambda topic, obj: log(topic, obj))
    else:
        # Add the topics that exist in *your* bus.
        for t in [
            "state.changed",
            "connection.lost",
            "connection.restored",
            "safety.triggered",
            "safety.cleared",
            "cmd.ack",
            "cmd.nack",
            "telemetry",
        ]:
            try:
                sub(t, lambda obj, tt=t: log(tt, obj))
            except Exception:
                pass

    def close():
        try:
            f.close()
        except Exception:
            pass
    return close


@pytest.fixture
def event_loop():
    """
    Fresh loop per test. Ensures no background tasks leak into the next test.
    This is THE most common cause of "random disarm/deactivate" during long HIL tests.
    """
    loop = asyncio.new_event_loop()
    try:
        yield loop
    finally:
        # Cancel anything still running
        pending = asyncio.all_tasks(loop)
        for t in pending:
            t.cancel()
        if pending:
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))

        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()