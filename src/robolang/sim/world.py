"""
World file, schema 1.0 (spec/simulation-v1.0.md section 3).

`load_world` reads the bytes of a world file and returns a validated, immutable `World`,
or raises `WorldError` for the first error found (codes E501 to E511, section 3.9).
Validation runs after the task is selected (3.8). Nothing here executes a task.
"""

from __future__ import annotations
import json
import re
from collections.abc import Mapping
from dataclasses import dataclass
from decimal import Decimal
from types import MappingProxyType

from .. import ir
from .values import DURATION_UNITS, SPEED_UNITS, Speed, duration_ms, parse_decimal

WORLD_SCHEMA = "robolang.sim.world"
WORLD_SCHEMA_VERSION = "1.0"
CLI_BINDINGS_SOURCE = "--bind"

_ID = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_FACT_KIND = {"robot_ready": "robot", "region_clear": "region", "sensor_available": "sensor"}
_BOM = b"\xef\xbb\xbf"


class WorldError(Exception):
    """A world-file error. `pointer` is an RFC 6901 JSON Pointer; "" is the whole document."""

    def __init__(self, code: str, pointer: str, message: str, filename: str = "<world>"):
        super().__init__(f"{code} at {pointer or '(document)'}: {message}")
        self.code = code
        self.pointer = pointer
        self.message = message
        self.filename = filename

    def format(self) -> str:
        """The diagnostic of spec section 2.4."""
        return f"{self.filename}: error[{self.code}] at {self.pointer or '(document)'}: {self.message}"


@dataclass(frozen=True)
class Region:
    id: str
    clear: bool | None


@dataclass(frozen=True)
class Robot:
    id: str
    ready: bool | None
    location: str
    gripper: bool
    max_speed: tuple[Speed, ...]


@dataclass(frozen=True)
class WorldObject:
    id: str
    present: bool | None
    location: str | None
    held_by: str | None


@dataclass(frozen=True)
class Sensor:
    name: str
    robot: str | None
    available: bool | None


@dataclass(frozen=True)
class Event:
    at_ms: Decimal
    fact: str
    target: str
    value: bool


@dataclass(frozen=True)
class TaskScript:
    name: str
    bindings: tuple[tuple[str, str], ...]  # in task parameter order
    faults: tuple[int, ...]  # sorted
    cancel_before_step: int | None


@dataclass(frozen=True)
class World:
    """A validated, read-only snapshot. Its mappings cannot be changed; execution works on a copy."""

    locations: tuple[str, ...]
    regions: Mapping[str, Region]
    robots: Mapping[str, Robot]
    objects: Mapping[str, WorldObject]
    sensors: Mapping[str, Sensor]
    events: tuple[Event, ...]  # stably sorted by at_ms: equal times keep file order
    task: TaskScript

    def kind_of(self, entity_id: str) -> str | None:
        if entity_id in self.locations:
            return "location"
        if entity_id in self.regions:
            return "region"
        if entity_id in self.robots:
            return "robot"
        if entity_id in self.objects:
            return "object"
        if entity_id in self.sensors:
            return "sensor"  # last: a sensor sharing an ID with an entity is read as the entity
        return None

    def binds(self, param_type: str, entity_id: str) -> bool:
        """True if a parameter of `param_type` may be bound to `entity_id` (a sensor parameter: any sensor)."""
        if param_type == "sensor":
            return entity_id in self.sensors
        return self.kind_of(entity_id) == param_type


# -------- reading the JSON --------

class _Float:
    """Stands in for any JSON number that is not an integer; numbers are only step indices."""


class _ReadError(Exception):
    pass


def _pairs(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise _ReadError(f"duplicate key {key!r}")
        out[key] = value
    return out


def _reject_constant(name):
    raise _ReadError(f"{name} is not allowed")


def _pointer(*parts) -> str:
    return "".join("/" + str(p).replace("~", "~0").replace("/", "~1") for p in parts)


class _Loader:
    def __init__(self, filename: str, task: ir.Task, cli_bindings: Mapping[str, str] | None = None):
        self.filename = filename
        self.task = task
        self.cli_bindings = cli_bindings

    def fail(self, code: str, pointer: str, message: str, filename: str | None = None):
        raise WorldError(code, pointer, message, self.filename if filename is None else filename)

    # -- document --

    def read(self, data: bytes):
        if data.startswith(_BOM):
            data = data[len(_BOM):]
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError as exc:
            self.fail("E501", "", f"not valid UTF-8: {exc.reason}")
        try:
            return json.loads(
                text, object_pairs_hook=_pairs, parse_constant=_reject_constant, parse_float=lambda s: _Float()
            )
        except _ReadError as exc:
            self.fail("E501", "", str(exc))
        except ValueError as exc:  # json.JSONDecodeError and over-long integers
            self.fail("E501", "", f"not valid JSON: {exc}")

    # -- form helpers --

    def obj(self, value, pointer, allowed, what):
        """`value` must be an object with no field outside `allowed`."""
        if not isinstance(value, dict):
            self.fail("E504", pointer, f"{what} must be an object")
        for key in value:
            if key not in allowed:
                self.fail("E503", pointer + _pointer(key), f"unknown field '{key}'")
        return value

    def need(self, value, pointer, name):
        if name not in value:
            self.fail("E504", pointer, f"missing required field '{name}'")
        return value[name]

    def boolean(self, value, pointer):
        if type(value) is not bool:
            self.fail("E504", pointer, "must be true or false")
        return value

    def optional_bool(self, parent, pointer, name):
        if name not in parent:
            return None
        return self.boolean(parent[name], pointer + _pointer(name))

    def string(self, value, pointer, what="value"):
        if not isinstance(value, str):
            self.fail("E504", pointer, f"{what} must be a string")
        return value

    def entity_id(self, value, pointer):
        if not isinstance(value, str):
            self.fail("E504", pointer, "an ID must be a string")
        if _ID.fullmatch(value) is None:
            self.fail("E504", pointer, f"not an entity ID: {value!r}")
        return value

    def step(self, value, pointer):
        if type(value) is not int or value < 1:
            self.fail("E504", pointer, "must be a positive integer")
        return value

    def quantity(self, value, pointer, units, what):
        q = self.obj(value, pointer, ("value", "unit"), what)
        text = self.need(q, pointer, "value")
        unit = self.need(q, pointer, "unit")
        try:
            number = parse_decimal(text)
        except ValueError:
            self.fail("E504", pointer + _pointer("value"), "must be a decimal string such as \"2.5\"")
        if not isinstance(unit, str) or unit not in units:
            self.fail("E504", pointer + _pointer("unit"), f"unit must be one of {', '.join(units)}")
        if number <= 0:
            self.fail("E504", pointer + _pointer("value"), "must be greater than zero")
        return number, unit

    def mapping(self, doc, name):
        """An optional collection of entities keyed by ID."""
        if name not in doc:
            return {}
        value = doc[name]
        pointer = _pointer(name)
        if not isinstance(value, dict):
            self.fail("E504", pointer, f"'{name}' must be an object")
        for key in value:
            self.entity_id(key, pointer + _pointer(key))
        return value

    # -- the whole file --

    def load(self, data: bytes) -> World:
        doc = self.read(data)
        if not isinstance(doc, dict):
            self.fail("E504", "", "the world file must be a JSON object")
        for name, expected in (("world_schema", WORLD_SCHEMA), ("world_schema_version", WORLD_SCHEMA_VERSION)):
            if name not in doc:
                self.fail("E502", "", f"missing '{name}'")
            if not isinstance(doc[name], str) or doc[name] != expected:
                self.fail("E502", _pointer(name), f"unsupported {name}: expected {expected!r}")
        top = (
            "world_schema", "world_schema_version", "locations", "regions", "robots",
            "objects", "sensors", "events", "tasks",
        )
        self.obj(doc, "", top, "the world file")
        locations = self.locations(self.need(doc, "", "locations"))
        regions_raw = self.mapping(doc, "regions")
        robots_raw = self.mapping(doc, "robots")
        objects_raw = self.mapping(doc, "objects")
        sensors_raw = doc.get("sensors", {})
        if not isinstance(sensors_raw, dict):
            self.fail("E504", "/sensors", "'sensors' must be an object")
        events_raw = doc.get("events", [])
        if not isinstance(events_raw, list):
            self.fail("E504", "/events", "'events' must be an array")
        tasks_raw = self.need(doc, "", "tasks")
        if not isinstance(tasks_raw, dict):
            self.fail("E504", "/tasks", "'tasks' must be an object")

        # Form of every section, in the document order of the fields above.
        regions = {k: self.region(k, v) for k, v in regions_raw.items()}
        robots = {k: self.robot(k, v) for k, v in robots_raw.items()}
        objects = {k: self.object(k, v) for k, v in objects_raw.items()}
        sensors = {k: self.sensor(k, v) for k, v in sensors_raw.items()}
        events = [self.event(i, v) for i, v in enumerate(events_raw)]
        entries = {k: self.task_entry(k, v) for k, v in tasks_raw.items()}

        # Identity: IDs are unique across the four kinds (3.2). Duplicates inside
        # `locations` were reported above; object keys cannot repeat (E501).
        seen = set(locations)
        for kind, ids in (("regions", regions), ("robots", robots), ("objects", objects)):
            for entity in ids:
                if entity in seen:
                    self.fail("E509", _pointer(kind, entity), f"duplicate entity ID '{entity}'")
                seen.add(entity)

        world = World(tuple(locations), regions, robots, objects, sensors, (), TaskScript(self.task.name, (), (), None))
        self.check_robots(world)
        self.check_objects(world)
        self.check_sensors(world)
        self.check_events(world, events)
        script = self.check_task(world, entries)
        ordered = tuple(sorted(events, key=lambda e: e.at_ms))  # stable: ties keep file order
        return World(
            world.locations,
            MappingProxyType(regions),
            MappingProxyType(robots),
            MappingProxyType(objects),
            MappingProxyType(sensors),
            ordered,
            script,
        )

    # -- sections: form --

    def locations(self, value):
        if not isinstance(value, list):
            self.fail("E504", "/locations", "'locations' must be an array")
        out = [self.entity_id(item, _pointer("locations", i)) for i, item in enumerate(value)]
        for i, item in enumerate(out):
            if item in out[:i]:
                self.fail("E509", _pointer("locations", i), f"duplicate entity ID '{item}'")
        return out

    def region(self, name, value):
        p = _pointer("regions", name)
        self.obj(value, p, ("clear",), "a region")
        return Region(name, self.optional_bool(value, p, "clear"))

    def robot(self, name, value):
        p = _pointer("robots", name)
        self.obj(value, p, ("ready", "location", "gripper", "max_speed"), "a robot")
        ready = self.optional_bool(value, p, "ready")
        location = self.entity_id(self.need(value, p, "location"), p + _pointer("location"))
        gripper = self.boolean(self.need(value, p, "gripper"), p + _pointer("gripper"))
        speeds = []
        if "max_speed" in value:
            mp = p + _pointer("max_speed")
            if not isinstance(value["max_speed"], list):
                self.fail("E504", mp, "'max_speed' must be an array")
            for i, item in enumerate(value["max_speed"]):
                number, unit = self.quantity(item, mp + _pointer(i), SPEED_UNITS, "a speed")
                speeds.append(Speed(number, unit))
        return Robot(name, ready, location, gripper, tuple(speeds))

    def object(self, name, value):
        p = _pointer("objects", name)
        self.obj(value, p, ("present", "location", "held_by"), "an object")
        present = self.optional_bool(value, p, "present")
        refs = []
        for field in ("location", "held_by"):
            if field in value:
                self.entity_id(value[field], p + _pointer(field))
            refs.append(value.get(field))
        return WorldObject(name, present, refs[0], refs[1])

    def sensor(self, name, value):
        p = _pointer("sensors", name)
        if name == "":
            self.fail("E504", p, "a sensor name must not be empty")
        self.obj(value, p, ("robot", "available"), "a sensor")
        robot = None
        if "robot" in value:
            robot = self.entity_id(value["robot"], p + _pointer("robot"))
        return Sensor(name, robot, self.optional_bool(value, p, "available"))

    def event(self, index, value):
        p = _pointer("events", index)
        self.obj(value, p, ("at", "fact", "target", "value"), "an event")
        at = self.need(value, p, "at")
        fact = self.need(value, p, "fact")
        target = self.need(value, p, "target")
        flag = self.need(value, p, "value")
        number, unit = self.quantity(at, p + _pointer("at"), tuple(DURATION_UNITS), "an event time")
        if not isinstance(fact, str) or fact not in _FACT_KIND:
            self.fail("E504", p + _pointer("fact"), f"fact must be one of {', '.join(_FACT_KIND)}")
        self.string(target, p + _pointer("target"), "target")
        self.boolean(flag, p + _pointer("value"))
        return Event(duration_ms(number, unit), fact, target, flag)

    def task_entry(self, name, value):
        p = _pointer("tasks", name)
        self.obj(value, p, ("bindings", "faults", "cancel"), "a task entry")
        bp = p + _pointer("bindings")
        if "bindings" not in value and self.cli_bindings is not None and name == self.task.name:
            raw = {}  # --bind replaces the bindings, so the selected task's entry may leave them out
        else:
            raw = self.need(value, p, "bindings")
        if not isinstance(raw, dict):
            self.fail("E504", bp, "'bindings' must be an object")
        for key, target in raw.items():
            self.string(target, bp + _pointer(key), "a binding")
        faults = []
        if "faults" in value:
            fp = p + _pointer("faults")
            if not isinstance(value["faults"], list):
                self.fail("E504", fp, "'faults' must be an array")
            for i, item in enumerate(value["faults"]):
                ip = fp + _pointer(i)
                self.obj(item, ip, ("step",), "a fault")
                faults.append((i, self.step(self.need(item, ip, "step"), ip + _pointer("step"))))
        cancel = None
        if "cancel" in value:
            cp = p + _pointer("cancel")
            self.obj(value["cancel"], cp, ("before_step",), "a cancellation")
            cancel = self.step(self.need(value["cancel"], cp, "before_step"), cp + _pointer("before_step"))
        return raw, faults, cancel

    # -- sections: meaning --

    def check_robots(self, world):
        for name, robot in world.robots.items():
            p = _pointer("robots", name)
            if world.kind_of(robot.location) != "location":
                self.fail("E505", p + _pointer("location"), f"'{robot.location}' is not a declared location")
            seen = set()
            for i, speed in enumerate(robot.max_speed):
                if speed.unit in seen:
                    self.fail("E511", p + _pointer("max_speed", i), f"second max_speed with unit '{speed.unit}'")
                seen.add(speed.unit)

    def check_objects(self, world):
        holding = {}
        for name, obj in world.objects.items():
            p = _pointer("objects", name)
            if obj.location is not None and world.kind_of(obj.location) != "location":
                self.fail("E505", p + _pointer("location"), f"'{obj.location}' is not a declared location")
            if obj.held_by is not None and world.kind_of(obj.held_by) != "robot":
                self.fail("E505", p + _pointer("held_by"), f"'{obj.held_by}' is not a declared robot")
            positions = [f for f, v in (("location", obj.location), ("held_by", obj.held_by)) if v is not None]
            if obj.present is True:
                if len(positions) != 1:
                    self.fail("E510", p, "a present object has exactly one of 'location' and 'held_by'")
            elif positions:
                self.fail("E510", p + _pointer(positions[0]), "an object not known to be present has no position")
            if obj.held_by is not None:
                if not world.robots[obj.held_by].gripper:
                    self.fail("E510", p + _pointer("held_by"), f"robot '{obj.held_by}' has no gripper")
                if obj.held_by in holding:
                    self.fail("E510", p + _pointer("held_by"), f"{obj.held_by} already holds '{holding[obj.held_by]}'")
                holding[obj.held_by] = name

    def check_sensors(self, world):
        for name, sensor in world.sensors.items():
            if sensor.robot is not None and world.kind_of(sensor.robot) != "robot":
                self.fail("E505", _pointer("sensors", name, "robot"), f"'{sensor.robot}' is not a declared robot")

    def check_events(self, world, events):
        for i, event in enumerate(events):
            p = _pointer("events", i, "target")
            expected = _FACT_KIND[event.fact]
            kind = world.kind_of(event.target)
            is_sensor = event.target in world.sensors  # sensors are named, not entities (3.2)
            if (is_sensor if expected == "sensor" else kind == expected):
                continue
            if kind is None and not is_sensor:
                self.fail("E505", p, f"'{event.target}' is not declared")
            self.fail("E511", p, f"'{event.fact}' needs a {expected}, not '{event.target}'")

    def check_task(self, world, entries):
        name = self.task.name
        p = _pointer("tasks", name)
        if self.cli_bindings is not None:
            # --bind replaces the entry's bindings entirely; its faults and cancel still apply.
            raw, faults, cancel = entries.get(name, ({}, [], None))
            bound = self.check_bindings(world, self.cli_bindings, _pointer("bindings"), CLI_BINDINGS_SOURCE)
        else:
            if name not in entries:
                self.fail("E506", "/tasks", f"no entry for task '{name}'")
                return None
            raw, faults, cancel = entries[name]
            bound = self.check_bindings(world, raw, p + _pointer("bindings"), self.filename)
        count = len(self.task.steps)
        seen = set()
        for i, step in faults:
            fp = p + _pointer("faults", i, "step")
            if step > count:
                self.fail("E511", fp, f"step {step} is outside the plan (1 to {count})")
                return None
            if step in seen:
                self.fail("E511", fp, f"step {step} is faulted twice")
                return None
            seen.add(step)
        if cancel is not None and cancel > count:
            self.fail("E511", p + _pointer("cancel", "before_step"), f"step {cancel} is outside 1 to {count}")
            return None
        return TaskScript(name, tuple(bound), tuple(sorted(seen)), cancel)

    def check_bindings(self, world, raw, bp, source):
        """Every parameter bound once to an entity of its type; no other names. `source` is named in diagnostics."""
        bound = []
        for parameter in self.task.parameters:
            if parameter.name not in raw:
                self.fail("E506", bp, f"parameter '{parameter.name}' is not bound", source)
                return bound
            target = raw[parameter.name]
            kind = world.kind_of(target)
            if kind == "sensor" and not self.task.typed_sensors:
                kind = None  # before RoboLang 1.3 a sensor cannot be bound: it stays undeclared
            if kind is None:
                self.fail("E505", bp + _pointer(parameter.name), f"'{target}' is not declared", source)
                return bound
            if not world.binds(parameter.type, target):
                self.fail(
                    "E507", bp + _pointer(parameter.name),
                    f"parameter '{parameter.name}' is a {parameter.type}, '{target}' is a {kind}",
                    source,
                )
                return bound
            bound.append((parameter.name, target))
        names = {parameter.name for parameter in self.task.parameters}
        for key in raw:
            if key not in names:
                self.fail("E508", bp + _pointer(key), f"'{key}' is not a parameter of task '{self.task.name}'", source)
                return bound
        return bound


def load_world(data: bytes, filename: str, task: ir.Task, bindings: Mapping[str, str] | None = None) -> World:
    """
    Validate a world file for the selected `task`. Raises WorldError on the first error.

    `bindings` (parameter name -> entity ID, from `--bind`) replaces the task entry's bindings
    entirely; the entry may then be absent. Its errors name `CLI_BINDINGS_SOURCE` as the file.
    """
    return _Loader(filename, task, bindings).load(data)
