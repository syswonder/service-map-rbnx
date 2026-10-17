# SPDX-License-Identifier: MulanPSL-2.0
"""Robot-time waits with real scheduling and cancellation checks."""

from contextlib import contextmanager
import asyncio
import logging
import math
import json
import os
import threading
import time

_clock = None
_config = None
_stopping = threading.Event()
_current = threading.local()


def configure(config, consumer):
    global _clock, _config
    config = dict(config or {})
    if _config is not None:
        if config != _config:
            raise ValueError("reference changed while mapping is initialized; restart mapping")
        return _clock.read() is not None
    from robonix_api._reference_time import _ReferenceClock
    _clock = _ReferenceClock(config, consumer)
    _config = config
    os.environ["MAPPING_REFERENCE_CONFIG"] = json.dumps(config)
    return _clock.read() is not None


def close():
    _stopping.set()
    if _clock is not None:
        _clock.close()


def reading():
    if _stopping.is_set():
        raise RuntimeError("mapping is shutting down")
    active = getattr(_current, "active", None)
    if active is not None and not active():
        raise RuntimeError("map operation canceled")
    value = _clock.read() if _clock is not None else (time.monotonic_ns(), 0, 0)
    if value is None:
        raise RuntimeError("Chronos reference unavailable")
    baseline = getattr(_current, "baseline", None)
    if baseline is not None and (value[1:] != baseline[1:] or value[0] < baseline[0]):
        raise RuntimeError("Chronos reference changed during map operation")
    return value


@contextmanager
def operation(active=None):
    previous = getattr(_current, "baseline", None), getattr(_current, "active", None)
    if active is not None:
        _current.active = active
    try:
        _current.baseline = reading()
        yield
        reading()
    finally:
        _current.baseline, _current.active = previous


@contextmanager
def grpc_operation(context):
    with operation(context.is_active):
        generation = dict(context.invocation_metadata()).get("robonix-reference-generation")
        if generation is not None and int(generation) != reading()[1]:
            raise RuntimeError("Chronos reference changed before map request arrived")
        yield


async def run(function, *args):
    active = threading.Event()
    active.set()

    def invoke():
        with operation(active.is_set):
            return function(*args)

    task = asyncio.create_task(asyncio.to_thread(invoke))
    try:
        return await asyncio.shield(task)
    except asyncio.CancelledError:
        active.clear()
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=5.0)
        except RuntimeError:
            pass  # The worker observes cancellation through operation().
        except asyncio.TimeoutError:
            logging.getLogger(__name__).error("map worker did not stop within 5 real seconds")
            task.add_done_callback(lambda done: done.exception() if not done.cancelled() else None)
        raise
    finally:
        active.clear()


class Deadline:
    def __init__(self, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("reference wait must be finite and nonnegative")
        self.seconds = seconds
        self.start = reading()

    def expired(self):
        now = reading()
        if now[1:] != self.start[1:] or now[0] < self.start[0]:
            raise RuntimeError("Chronos reference changed during map wait")
        return (now[0] - self.start[0]) / 1e9 >= self.seconds


def wait(event, seconds):
    deadline = Deadline(seconds)
    while True:
        expired = deadline.expired()
        if event.is_set():
            return True
        if expired:
            return False
        event.wait(0.05)
