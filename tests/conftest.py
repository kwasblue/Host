import pytest

from helpers import FakeAsyncTransport, CapturingBus


@pytest.fixture
def bus():
    return CapturingBus()


@pytest.fixture
def transport():
    return FakeAsyncTransport(auto_ack=True)
