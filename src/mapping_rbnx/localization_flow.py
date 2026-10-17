# SPDX-License-Identifier: MulanPSL-2.0
"""One initial-localization run: search, choose/filter, verify, hand over.

The controller owns results and transitions. HTTP/RPC clients only read its
snapshot or submit a pose; they never launch another global search. A restart
cancels the previous run and serializes ROS side effects before the new run.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import copy
import threading
import time

from mapping_rbnx import reference_time


class Cancelled(Exception):
    pass


@dataclass
class Run:
    run_id: int
    map_id: str
    generation: int
    seed: tuple | None = None
    cancelled: threading.Event = field(default_factory=threading.Event)
    signal: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None
    reference: tuple = field(default_factory=reference_time.reading)

    def check(self):
        if self.cancelled.is_set():
            raise Cancelled()
        current = reference_time.reading()
        if current[1:] != self.reference[1:] or current[0] < self.reference[0]:
            raise RuntimeError("Chronos reference changed during localization")


class LocalizationFlow:
    def __init__(self):
        self._lock = threading.RLock()
        self._execution = threading.Lock()
        self._run = None
        self._workers = set()
        self._state = {"run_id": 0, "state": "idle", "phase": "idle",
                       "map_id": "", "generation": 0, "candidates": [],
                       "detail": "", "filter_state": "off", "started_at": 0.0,
                       "phase_started_at": 0.0}

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._state)

    def cancel(self):
        """Drain a localization run before an explicit mapping/reset action."""
        with self._lock:
            if self._run:
                self._run.cancelled.set()
                self._run.signal.set()
            self._state.update(state="idle", phase="idle", candidates=[], filter_state="off")
            workers = list(self._workers)
        deadline = time.monotonic() + 8.0
        for worker in workers:
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
            if worker.is_alive():
                raise RuntimeError("localization worker did not stop")

    def _update(self, run, phase=None, **values):
        with self._lock:
            if phase != "failed":
                run.check()
            elif run.cancelled.is_set():
                raise Cancelled()
            if self._run is not run:
                raise Cancelled()
            if phase is not None:
                self._state.update(phase=phase, phase_started_at=time.time(),
                                   state={"ready": "done", "failed": "failed"}.get(phase, "running"))
                if phase not in {"choosing", "failed"}:
                    self._state["candidates"] = []
            self._state.update(values)

    def start(self, backend, map_id, generation, seed=None):
        with self._lock:
            self._workers = {worker for worker in self._workers if worker.is_alive()}
            if len(self._workers) >= 8:
                raise RuntimeError("localization workers are busy; retry after cancellation")
            previous = self._run
            if previous:
                previous.cancelled.set()
                previous.signal.set()
            run = Run(self._state["run_id"] + 1, map_id, generation, seed)
            self._run = run
            now = time.time()
            self._state = {"run_id": run.run_id, "map_id": map_id,
                           "generation": generation, "state": "running",
                           "phase": "restarting" if previous and previous.thread.is_alive() else "searching",
                           "detail": "Starting initial localization", "candidates": [],
                           "filter_state": "off", "started_at": now, "phase_started_at": now}
            run.thread = threading.Thread(target=self._work, args=(run, backend),
                                          name=f"localize-{run.run_id}", daemon=True)
            self._workers.add(run.thread)
            run.thread.start()
            return run.run_id

    def submit(self, pose, run_id=None):
        with self._lock:
            run = self._run
            if run is None or (run_id is not None and run.run_id != run_id):
                return False, "localization run changed; refresh before choosing a pose"
            if self._state["phase"] not in {"searching", "choosing", "restarting"}:
                return False, "initial poses are accepted only while searching or choosing"
            run.check()
            run.seed = tuple(pose)
            run.signal.set()
            # Stop offering alternatives as soon as a selection is accepted.
            self._state["candidates"] = []
            self._state.update(phase="verifying", phase_started_at=time.time())
            self._state["detail"] = "Initial pose received; waiting for laser verification"
            return True, "initial pose received; checking it against the saved map"

    def _selected(self, run):
        with self._lock:
            run.check()
            return run.seed

    def _work(self, run, backend):
        # An obsolete CPU search may finish, but cannot publish results or
        # start an engine. No two runs manipulate the ROS processes together.
        with self._execution:
            try:
                with reference_time.operation(lambda: not run.cancelled.is_set()):
                    run.check()
                    backend.prepare(run)
                    run.check()
                    pose = self._selected(run)
                    detail = "Using the supplied initial pose"
                    if pose is None:
                        self._update(run, "searching", detail="Matching the scan against the saved map")
                        match, detail = backend.match(run)
                        run.check()
                        pose = self._selected(run)
                        if pose is None and match is not None and backend.conclusive(match):
                            pose, detail = match.pose, f"Clear automatic match: {match.detail}"
                        if pose is None:
                            candidates = [dict(rank=i, x=float(c.pose[0]), y=float(c.pose[1]),
                                               theta=float(c.pose[2]), score=float(c.score))
                                          for i, c in enumerate(match.candidates[:5], 1)] if match else []
                            with self._lock:
                                pose = self._selected(run)
                                if pose is None:
                                    self._update(run, "choosing", candidates=candidates,
                                                 detail="Choose a pose or move the robot manually to help the particle filter converge",
                                                 filter_state="starting")
                            if pose is None:
                                pose, detail = backend.filter(run)
                            run.check()
                            pose = self._selected(run) or pose
                            if pose is None:
                                self._update(run, filter_state="stopped", detail=detail + "; choose an initial pose or restart localization")
                                # The user can still choose after filter timeout.
                                while not run.signal.wait(0.05):
                                    run.check()
                                run.check()
                                pose = self._selected(run)
                        detail = "Using the operator's initial pose" if self._selected(run) else detail
                    run.check()
                    with self._lock:
                        # Atomically close selection before claiming a pose; a
                        # simultaneous filter result and user choice has one winner.
                        pose = self._selected(run) or pose
                        self._update(run, "verifying", detail="Verifying the initial pose against the live laser", filter_state="off")
                    backend.stop_filter()
                    ok, verified = backend.verify(run, pose)
                    run.check()
                    if not ok:
                        raise RuntimeError(verified)
                    self._update(run, "starting", detail="Starting the map engine at the verified pose")
                    ok, handover = backend.handover(run, pose)
                    run.check()
                    if not ok:
                        raise RuntimeError(handover)
                    self._update(run, "ready", detail=f"{detail}; {verified}; {handover}")
            except Cancelled:
                pass
            except Exception as exc:
                try:
                    self._update(run, "failed", detail=str(exc), filter_state="off", candidates=[])
                except Cancelled:
                    pass
            finally:
                state = self.snapshot()
                backend.cleanup(run, state["run_id"] == run.run_id and state["phase"] == "ready")

    def filter_running(self, run):
        self._update(run, filter_state="running")
