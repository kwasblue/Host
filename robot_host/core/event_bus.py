from collections import defaultdict
from typing import Callable, Any, Dict, List


class EventBus:
    def __init__(self) -> None:
        self._subscribers: Dict[str, List[Callable[[Any], None]]] = defaultdict(list)

    def subscribe(self, event_type: str, handler: Callable[[Any], None]) -> None:
        self._subscribers[event_type].append(handler)

    def publish(self, event_type: str, payload: Any = None) -> None:
        for h in self._subscribers[event_type]:
            h(payload)
