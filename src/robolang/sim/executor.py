"""
Execution of a task against a world (spec/simulation-v1.0.md section 4).

`run` is pure and deterministic: no wall clock, no randomness, no I/O. It copies the facts of
the `World` into private state and never mutates the `World`. The result holds the data of the
run record (section 5); serializing it is not done here.
"""

from __future__ import annotations
from dataclasses import dataclass
from decimal import Decimal

from .. import ir
from .values import Truth, duration_ms_of
from .world import World


@dataclass(frozen=True)
class Failure:
    code: str
    phase: str  # "authorization", "precondition" or "step"
    step: int | None
    precondition: int | None
    safety_limit: int | None
    robot: str | None


@dataclass(frozen=True)
class LimitResult:
    limit: ir.SafetyLimit
    status: str  # unit_unspecified_not_enforced, checked_against_profile_not_enforced, failed_check, not_checked


@dataclass(frozen=True)
class PreconditionResult:
    index: int
    condition: ir.Condition
    result: Truth | None  # None: not evaluated


@dataclass(frozen=True)
class Observation:
    object: str
    present: bool | None
    location: str | None
    held_by: str | None


@dataclass(frozen=True)
class Message:
    robot: str
    channel: str
    message: str
    status: str = "recorded"


@dataclass(frozen=True)
class StepResult:
    index: int
    step: ir.Step
    status: str  # succeeded, failed, not_run
    t_start_ms: Decimal | None
    t_end_ms: Decimal | None
    observation: Observation | None
    message: Message | None
    condition_result: Truth | None


@dataclass(frozen=True)
class AppliedEvent:
    t_ms: Decimal
    fact: str
    target: str
    value: bool


@dataclass(frozen=True)
class RobotState:
    ready: bool | None
    location: str
    holding: str | None


@dataclass(frozen=True)
class ObjectState:
    present: bool | None
    location: str | None
    held_by: str | None


@dataclass(frozen=True)
class RegionState:
    clear: bool | None


@dataclass(frozen=True)
class SensorState:
    robot: str | None
    available: bool | None


@dataclass(frozen=True)
class FinalWorld:
    """Each field is a tuple of (id, state) pairs sorted by ID (section 5.4)."""

    robots: tuple[tuple[str, RobotState], ...]
    objects: tuple[tuple[str, ObjectState], ...]
    regions: tuple[tuple[str, RegionState], ...]
    sensors: tuple[tuple[str, SensorState], ...]


@dataclass(frozen=True)
class RunResult:
    task_name: str
    bindings: tuple[tuple[str, str], ...]
    states: tuple[str, ...]
    state: str
    failure: Failure | None
    cancelled_before_step: int | None
    safety_limits: tuple[LimitResult, ...]
    preconditions: tuple[PreconditionResult, ...]
    steps: tuple[StepResult, ...]
    events_applied: tuple[AppliedEvent, ...]
    end_t_ms: Decimal
    final_world: FinalWorld


class SimulationInputError(ValueError):
    """`run` was given a task and a world that do not belong together. Nothing was run."""


class _StepFailed(Exception):
    def __init__(self, code: str, condition_result: Truth | None = None):
        super().__init__(code)
        self.code = code
        self.condition_result = condition_result  # a `wait until` that timed out records its last value


class _State:
    """The private mutable copy of the world facts."""

    def __init__(self, world: World):
        self.world = world
        self.ready = {k: v.ready for k, v in world.robots.items()}
        self.location = {k: v.location for k, v in world.robots.items()}
        self.holding: dict[str, str | None] = {k: None for k in world.robots}
        self.present = {k: v.present for k, v in world.objects.items()}
        self.at = {k: v.location for k, v in world.objects.items()}
        self.held_by = {k: v.held_by for k, v in world.objects.items()}
        for name, holder in self.held_by.items():
            if holder is not None:
                self.holding[holder] = name
        self.clear = {k: v.clear for k, v in world.regions.items()}
        self.available = {k: v.available for k, v in world.sensors.items()}
        self.clock = Decimal(0)
        self.next_event = 0
        self.applied: list[AppliedEvent] = []

    def advance(self, t: Decimal) -> None:
        """Set the clock to `t` and apply every event with at <= t not yet applied (4.2)."""
        self.clock = t
        events = self.world.events
        while self.next_event < len(events) and events[self.next_event].at_ms <= t:
            event = events[self.next_event]
            self.next_event += 1
            if event.fact == "robot_ready":
                self.ready[event.target] = event.value
            elif event.fact == "region_clear":
                self.clear[event.target] = event.value
            else:
                self.available[event.target] = event.value
            self.applied.append(AppliedEvent(event.at_ms, event.fact, event.target, event.value))

    def freeze(self) -> FinalWorld:
        w = self.world
        return FinalWorld(
            tuple((k, RobotState(self.ready[k], self.location[k], self.holding[k])) for k in sorted(w.robots)),
            tuple((k, ObjectState(self.present[k], self.at[k], self.held_by[k])) for k in sorted(w.objects)),
            tuple((k, RegionState(self.clear[k])) for k in sorted(w.regions)),
            tuple((k, SensorState(w.sensors[k].robot, self.available[k])) for k in sorted(w.sensors)),
        )


def _require_ready(ready: bool | None) -> None:
    if ready is not True:
        raise _StepFailed("S202" if ready is None else "S201")


def _evaluate(state: _State, bound: dict[str, str], condition: ir.Condition) -> Truth:
    if isinstance(condition, ir.RobotReady):
        return Truth.of(state.ready[bound[condition.robot]])
    if isinstance(condition, ir.RegionClear):
        return Truth.of(state.clear[bound[condition.region]])
    if isinstance(condition, ir.ObjectExists):
        return Truth.of(state.present[bound[condition.obj]])
    if condition.sensor in state.available:
        return Truth.of(state.available[condition.sensor])
    return Truth.UNKNOWN  # no sensor of that name is declared


def _authorize(task: ir.Task, world: World, bindings: tuple[tuple[str, str], ...]):
    """Section 4.3. Returns (limit results, Failure or None)."""
    bound = dict(bindings)
    robots = [bound[p.name] for p in task.parameters if p.type == "robot"]
    statuses: list[str] = []
    failure = None
    for index, limit in enumerate(task.safety, start=1):
        if failure is not None:
            statuses.append("not_checked")
            continue
        if limit.limit.unit is None:
            statuses.append("unit_unspecified_not_enforced")
            continue
        code, culprit = None, None
        for robot in robots:
            speeds = {s.unit: s.value for s in world.robots[robot].max_speed}
            if limit.limit.unit not in speeds:
                code, culprit = "S002", robot
            elif limit.limit.value > speeds[limit.limit.unit]:
                code, culprit = "S001", robot
            if code is not None:
                break
        if code is None:
            statuses.append("checked_against_profile_not_enforced")
        else:
            statuses.append("failed_check")
            failure = Failure(code, "authorization", None, None, index, culprit)
    return tuple(LimitResult(lim, status) for lim, status in zip(task.safety, statuses)), failure


def _execute(state: _State, bound: dict[str, str], step: ir.Step):
    """Perform one step. Returns (observation, message, condition_result). Raises _StepFailed."""
    if isinstance(step, ir.Move):
        robot = bound[step.robot]
        _require_ready(state.ready[robot])
        state.location[robot] = bound[step.target]  # a held object has no location of its own
    elif isinstance(step, ir.Grasp):
        robot, obj = bound[step.robot], bound[step.obj]
        _require_ready(state.ready[robot])
        if not state.world.robots[robot].gripper:
            raise _StepFailed("S211")
        if state.holding[robot] is not None:
            raise _StepFailed("S212")
        if state.present[obj] is not True:
            raise _StepFailed("S214" if state.present[obj] is None else "S213")
        if state.at[obj] != state.location[robot]:
            raise _StepFailed("S215")
        state.at[obj] = None
        state.held_by[obj] = robot
        state.holding[robot] = obj
    elif isinstance(step, ir.Place):
        robot, obj, target = bound[step.robot], bound[step.obj], bound[step.target]
        _require_ready(state.ready[robot])
        if state.holding[robot] != obj:
            raise _StepFailed("S221")
        if state.location[robot] != target:
            raise _StepFailed("S222")
        state.at[obj] = target
        state.held_by[obj] = None
        state.holding[robot] = None
    elif isinstance(step, ir.Inspect):
        robot, obj = bound[step.robot], bound[step.obj]
        sensor = state.world.sensors.get(step.sensor)
        if sensor is None or sensor.robot != robot:
            raise _StepFailed("S231")
        available = state.available[step.sensor]
        if available is not True:
            raise _StepFailed("S233" if available is None else "S232")
        return Observation(obj, state.present[obj], state.at[obj], state.held_by[obj]), None, None
    elif isinstance(step, ir.WaitFor):
        state.advance(state.clock + duration_ms_of(step.duration))
    elif isinstance(step, ir.WaitUntil):
        t0 = state.clock
        deadline = t0 + duration_ms_of(step.timeout)
        value = _evaluate(state, bound, step.condition)
        while value is not Truth.TRUE:
            pending = [e.at_ms for e in state.world.events[state.next_event:] if t0 < e.at_ms <= deadline]
            if not pending:
                state.advance(deadline)
                raise _StepFailed("S241", value)
            state.advance(pending[0])
            value = _evaluate(state, bound, step.condition)
        return None, None, value
    else:
        return None, Message(bound[step.robot], step.channel, step.message), None
    return None, None, None


def _not_run(task: ir.Task, first: int = 1) -> list[StepResult]:
    return [
        StepResult(i, s, "not_run", None, None, None, None, None)
        for i, s in enumerate(task.steps, start=1)
        if i >= first
    ]


def _names(node) -> list[str]:
    """The parameter names a condition or step refers to."""
    if isinstance(node, ir.WaitUntil):
        return _names(node.condition)
    return [getattr(node, f) for f in ("robot", "region", "obj", "target") if hasattr(node, f)]


def _check_inputs(task: ir.Task, world: World) -> None:
    """The trust boundary of `run`: the world must have been validated for this very task."""
    if not isinstance(task, ir.Task) or not isinstance(world, World):
        raise SimulationInputError("run() needs an ir.Task and a World from load_world")
    if world.task.name != task.name:
        raise SimulationInputError(f"the world was validated for task '{world.task.name}', not '{task.name}'")
    bound = dict(world.task.bindings)
    if [name for name, _ in world.task.bindings] != [p.name for p in task.parameters]:
        raise SimulationInputError(f"the world's bindings do not match the parameters of task '{task.name}'")
    for parameter in task.parameters:
        if world.kind_of(bound[parameter.name]) != parameter.type:
            raise SimulationInputError(f"parameter '{parameter.name}' is not bound to a {parameter.type}")
    for node in (*task.preconditions, *task.steps):
        for name in _names(node):
            if name not in bound:
                raise SimulationInputError(f"task '{task.name}' refers to '{name}', which is not a parameter")
    script = world.task
    for index in (*script.faults, *(() if script.cancel_before_step is None else (script.cancel_before_step,))):
        if not 1 <= index <= len(task.steps):
            raise SimulationInputError(f"the world scripts step {index}, outside the plan of task '{task.name}'")


def run(task: ir.Task, world: World) -> RunResult:
    """
    Run `task` against `world` in simulation (section 4). Never mutates `world`.

    `task` must come from the compiler (`compile_source`) and `world` from `load_world` for that
    same task, as `robolang simulate` does. `run` re-checks that they belong together and raises
    `SimulationInputError` otherwise, before anything runs.
    """
    _check_inputs(task, world)
    bindings = world.task.bindings
    bound = dict(bindings)
    limits, authorization = _authorize(task, world, bindings)
    state = _State(world)
    if authorization is not None:
        return RunResult(
            task.name, bindings, ("CREATED", "VALIDATED", "FAILED"), "FAILED", authorization, None, limits,
            tuple(PreconditionResult(i, c, None) for i, c in enumerate(task.preconditions, start=1)),
            tuple(_not_run(task)), (), state.clock, state.freeze(),
        )

    states = ["CREATED", "VALIDATED", "AUTHORIZED", "RUNNING"]
    failure = None
    cancelled = None

    pre = tuple(
        PreconditionResult(i, c, _evaluate(state, bound, c)) for i, c in enumerate(task.preconditions, start=1)
    )
    for p in pre:
        if p.result is not Truth.TRUE:
            failure = Failure("S101" if p.result is Truth.FALSE else "S102", "precondition", None, p.index, None, None)
            break

    results: list[StepResult] = []
    if failure is not None:
        results = _not_run(task)
    else:
        for i, step in enumerate(task.steps, start=1):
            if world.task.cancel_before_step == i:
                cancelled = i
                results.extend(_not_run(task, i))
                break
            start = state.clock
            try:
                if i in world.task.faults:
                    raise _StepFailed("S251")
                observation, message, condition_result = _execute(state, bound, step)
            except _StepFailed as exc:
                failure = Failure(exc.code, "step", i, None, None, None)
                results.append(StepResult(i, step, "failed", start, state.clock, None, None, exc.condition_result))
                results.extend(_not_run(task, i + 1))
                break
            results.append(
                StepResult(i, step, "succeeded", start, state.clock, observation, message, condition_result)
            )

    states.append("FAILED" if failure is not None else "CANCELLED" if cancelled is not None else "SUCCEEDED")
    return RunResult(
        task.name, bindings, tuple(states), states[-1], failure, cancelled, limits, pre, tuple(results),
        tuple(state.applied), state.clock, state.freeze(),
    )
