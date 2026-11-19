"""
Running a mission in simulation (RoboLang 2.0, decision 0041; spec/simulation-v1.0.md section 18).

A mission is run on the planner's side: it starts task runs one after another in ONE world with continuous time
(clarification 9), and every task run is a complete, verified run exactly as `executor.run` makes it (its own `pre`,
limits, steps and goal). The mission decides only whether to start the next run, and it never claims more than its
runs verified (M5). `run_mission` is pure and deterministic, like `run`.
"""

from __future__ import annotations
import dataclasses
from collections.abc import Container
from dataclasses import dataclass
from decimal import Decimal

from .. import ir
from ..tokens import MAX_MISSION_RUNS
from .messages import ReceivedMessage, received, undeclared_in
from .executor import AppliedEvent, FinalWorld, RunResult, SimulationInputError, _State, _StepFailed, _evaluate, run
from .values import Truth, duration_ms_of
from .world import TaskScript, World

COMPLETED, BOUNDED, FAILED, STOPPED = "completed", "bounded", "failed", "stopped"  # M5
S401 = "S401"  # a task cannot fit the duration bound even once (clarification 8): the mission fails before any run
S402 = "S402"  # the `until` condition is unknown (M3, fails closed): the mission stops
S011 = "S011"  # RoboLang 2.1: the `until` names a channel the world does not declare: the mission fails before any run
NO_PROGRESS = "no_progress"  # RoboLang 2.3: a failed run left the world and the clock as found, under a time bound


@dataclass(frozen=True)
class MissionFailure:
    """Why a mission is `failed` or `stopped`: `code` is the failed run's own code, or S401 or S402."""

    code: str
    run: int | None  # the number of the run it concerns (the last failed run), or None before any run
    message: str | None = None


@dataclass(frozen=True)
class TaskRun:
    index: int  # 1-based, over every task run of the mission (a retry is a run, clarification 7)
    line: int  # the 1-based place of its `run` line in the mission
    attempt: int  # 1, or 2 and more for a retry
    t_start_ms: Decimal  # mission time
    task: ir.Task
    bindings: tuple[tuple[str, str], ...]
    result: RunResult
    until: str | None  # "true", "false" or "unknown" after a run that was checked, else None

    @property
    def verified(self) -> bool:
        return self.result.state == "SUCCEEDED"


@dataclass(frozen=True)
class MissionResult:
    mission: ir.Mission
    bindings: tuple[tuple[str, str], ...]
    outcome: str  # completed, bounded, failed or stopped
    ended_by: str  # until, run_bound, duration_bound, run_failed, retries_used, bound_with_retry, all_failed,
    #                until_unknown, cannot_fit, undeclared_channel or no_progress
    failure: MissionFailure | None
    runs: tuple[TaskRun, ...]
    events_applied: tuple[AppliedEvent, ...]
    end_t_ms: Decimal
    final_world: FinalWorld
    messages: tuple[ReceivedMessage, ...] = ()  # RoboLang 2.1: every message since the mission started

    @property
    def runs_verified(self) -> int:
        return sum(1 for r in self.runs if r.verified)

    @property
    def runs_failed(self) -> int:
        return len(self.runs) - self.runs_verified


def _time_bound(condition: ir.Condition | None) -> bool:
    """`clock_time >= T` (or `>`), alone or as one operand of an `or`: an `until` that is a bound (decision 0041)."""
    if isinstance(condition, ir.Or):
        return any(_time_bound(c) for c in condition.conditions)
    return (isinstance(condition, ir.Compare) and isinstance(condition.left, ir.LiveClock)
            and condition.left.reading == "time" and condition.op in (">", ">=")
            and isinstance(condition.right, ir.TimeLit))


def mission_task(mission: ir.Mission) -> ir.Task:
    """
    The mission as a task with no steps, which is what `load_world` binds: a world file binds a mission's parameters
    in `tasks.<mission>.bindings`, or `--bind` does, as it does for a task. It is never run.
    """
    return ir.Task(mission.name, mission.parameters, (), (), (), steps=())


def min_task_ms(task: ir.Task) -> Decimal:
    """
    The least time a run of `task` that succeeds can take, from its `wait for` steps alone (a `wait until` may end at
    once, a move takes no simulated time): the shortest path through the branches, one that ends in `fail` not counted.
    """
    def shortest(steps: tuple[ir.Step, ...]) -> Decimal | None:
        total = Decimal(0)
        for step in steps:
            if isinstance(step, ir.Fail):
                return None
            if isinstance(step, ir.WaitFor):
                total += duration_ms_of(step.duration)
            elif isinstance(step, ir.If):
                options = [t for t in (shortest(step.then), shortest(step.orelse)) if t is not None]
                if not options:
                    return None
                total += min(options)
        return total

    return shortest(task.steps) or Decimal(0)


def _faulted_step(task: ir.Task) -> int:
    """The step a test-form fault of a whole run is put on: the first one that is not an `if`."""
    return next(i for i, (_, step) in enumerate(ir.walk(task.steps), start=1) if not isinstance(step, ir.If))


def run_mission(document: ir.TaskIR, mission: ir.Mission, world: World, *,
                fail_runs: Container[int] | str = ()) -> MissionResult:
    """
    Run `mission` against `world` (one loaded for `mission_task(mission)`), starting its task runs from `document`.

    `fail_runs` is a test form, not a world field: the numbers of the task runs that fail ("all": every run). A failed
    run is a scripted fault on the first step of its task (S251), which changes nothing in the world.
    """
    if world.task.name != mission.name or [n for n, _ in world.task.bindings] != [p.name for p in mission.parameters]:
        raise SimulationInputError(f"the world was not loaded for mission '{mission.name}'")
    bound = dict(world.task.bindings)
    pseudo = mission_task(mission)
    tasks = []
    for line in mission.runs:
        task = document.task(line.task)
        if task is None or len(line.arguments) != len(task.parameters):
            raise SimulationInputError(f"mission '{mission.name}' runs '{line.task}', which does not match a task")
        tasks.append(task)
    repeat, failure_policy = mission.repeat, mission.on_failure
    max_runs = None if repeat is None else repeat.max_runs
    max_ms = None if repeat is None or repeat.max_duration is None else duration_ms_of(repeat.max_duration)
    until = None if repeat is None else repeat.until
    # RoboLang 2.3 (decision 0045 C10, open point 3): under a time bound, `continue` after a run that changed neither
    # the world nor the clock would repeat that run for ever (a choice that finds no candidate fails at once), so it
    # stops
    from_2_3 = tuple(map(int, document.language_version.split("."))) >= (2, 3)
    timed = (max_ms is not None or _time_bound(until)) and from_2_3
    runs: list[TaskRun] = []
    state = _State(world)

    def result(outcome: str, ended_by: str, failure: MissionFailure | None = None) -> MissionResult:
        return MissionResult(mission, world.task.bindings, outcome, ended_by, failure, tuple(runs),
                             tuple(state.applied), state.clock, state.freeze(),
                             received(state.incoming, world, Decimal(0), state.clock))

    missing = undeclared_in(until, world)  # RoboLang 2.1 (decision 0043 S4, C1): fails before any run
    if missing:
        declared = ", ".join(f"'{c}'" for c in sorted(world.channels))
        return result(FAILED, "undeclared_channel", MissionFailure(
            S011, None, f"the until condition names channel '{missing[0]}', which the world does not declare "
                        f"(channels: {declared})"))

    if max_ms is not None:  # clarification 8: a task that cannot fit even once fails before the run
        for task in tasks:
            if min_task_ms(task) > max_ms:
                return result(FAILED, "cannot_fit", MissionFailure(
                    S401, None, f"task '{task.name}' needs at least {ir.decimal_text(min_task_ms(task) / 1000)} s from "
                                f"its waits and the duration bound is {ir.decimal_text(max_ms / 1000)} s"))

    def bound_reached() -> str | None:
        if len(runs) >= (MAX_MISSION_RUNS if max_runs is None else min(max_runs, MAX_MISSION_RUNS)):
            return "run_bound"
        if max_ms is not None and state.clock >= max_ms:
            return "duration_bound"
        return None

    def last_failure() -> MissionFailure | None:
        failed = [r for r in runs if not r.verified]
        if not failed:
            return None
        return MissionFailure(failed[-1].result.failure.code, failed[-1].index)

    def until_value() -> Truth:
        state.message_origin = Decimal(0)  # C8: the until sees every message since the mission started
        try:
            return _evaluate(pseudo, state, bound, until)
        except _StepFailed:  # a clock reading that cannot be taken (S237, S238) is unknown, as in a comparison
            return Truth.UNKNOWN

    while True:  # one pass through the `run` lines per turn
        for place, (line, task) in enumerate(zip(mission.runs, tasks), start=1):
            attempt = 1
            while True:
                reached = bound_reached() if repeat is not None else None
                if reached is None and len(runs) >= MAX_MISSION_RUNS:
                    reached = "run_bound"
                if reached is not None:
                    if attempt > 1:  # a retry was pending: the mission never claims what no run verified (M5)
                        return result(FAILED, "bound_with_retry", last_failure())
                    if runs and not any(r.verified for r in runs):
                        return result(FAILED, "all_failed", last_failure())
                    return result(BOUNDED, reached)
                number = len(runs) + 1
                bindings = tuple((p.name, bound[a]) for p, a in zip(task.parameters, line.arguments))
                faults = (_faulted_step(task),) if fail_runs == "all" or number in fail_runs else ()
                run_world = dataclasses.replace(world, task=TaskScript(task.name, bindings, faults, None))
                started_at = state.clock
                before = state.freeze() if timed else None
                outcome = run(task, run_world, state)
                entry = TaskRun(number, place, attempt, started_at, task, bindings, outcome, None)
                runs.append(entry)
                if outcome.state == "SUCCEEDED":
                    if until is not None:
                        value = until_value()
                        runs[-1] = dataclasses.replace(entry, until=value.value)
                        if value is Truth.TRUE:
                            return result(COMPLETED, "until")
                        if value is Truth.UNKNOWN:
                            return result(FAILED, "until_unknown", MissionFailure(
                                S402, number, "the until condition is unknown after this run, so the mission stops"))
                    break
                if failure_policy.policy == "stop":
                    return result(FAILED, "run_failed", last_failure())
                if failure_policy.policy == "retry":
                    if attempt > failure_policy.retries:
                        return result(FAILED, "retries_used", last_failure())
                    attempt += 1
                    continue
                if timed and state.clock == started_at and state.freeze() == before:
                    return result(FAILED, NO_PROGRESS, last_failure())  # the next run would be this one again
                break  # continue: the failed run is recorded, never counted as done, and the mission goes on
        if repeat is None:
            if not any(r.verified for r in runs):
                return result(FAILED, "all_failed", last_failure())
            return result(COMPLETED, "run_verified")
