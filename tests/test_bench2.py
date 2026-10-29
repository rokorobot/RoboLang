"""
BENCH-2 (decision record 0025): the expected outcome of every case is checked mechanically where it can be
(design §5), the scorer puts synthetic answers in each outcome class, and the runner works end to end with a stand-in
client. No network.
"""

import io
import json
from collections import Counter

import pytest

from bench import lib as v1
from bench.v2 import lib, run, score

CASES = lib.load_cases()


def ref_answer(name: str) -> dict:
    return {"kind": "program", **lib.reference(name), "category": "none", "construct": "none", "slot": "none",
            "entities": [], "message": ""}


def other(kind: str, category="none", construct="none", slot="none", entities=()) -> dict:
    return {"kind": kind, "rob": "", "task": "", "bindings": {}, "category": category, "construct": construct,
            "slot": slot, "entities": list(entities), "message": "?"}


def readings(case_id: str) -> list[str]:
    return sorted(p.stem for p in lib.REFERENCE_DIR.glob(f"{case_id}.*.json"))


def run_in(case: dict, answer: dict, patch: dict | None = None):
    data = None if patch is None else json.dumps(v1._merge(json.loads(v1.world_bytes(case)), patch)).encode()
    checked = v1.check_answer(case, answer, data)
    return checked, (v1.simulate_task(checked.task, checked.world) if checked.valid else None)


def test_the_case_set():
    assert Counter(c["class"] for c in CASES.values()) == {k: 20 for k in lib.CLASSES}
    assert sum("decoy" in c["tags"] for c in CASES.values()) == 8
    for path in (lib.V2 / "worlds").glob("*.json"):
        world = json.loads(path.read_text(encoding="utf-8"))
        size = len(world["locations"]) + sum(len(world[k]) for k in ("regions", "robots", "objects", "sensors"))
        assert size >= 25, path
    for case in CASES.values():
        assert case["prompt"].endswith((".", "?")) and case["tags"]


@pytest.mark.parametrize("case_id", sorted(c for c in CASES if CASES[c]["class"] == "feasible"))
def test_feasible_reference_is_correct(case_id):
    assert lib.classify(CASES[case_id], ref_answer(case_id))["outcome"] == "correct"


@pytest.mark.parametrize("case_id", sorted(c for c in CASES if CASES[c]["class"] == "infeasible"))
def test_infeasible_faithful_reference_fails_and_its_fix_succeeds(case_id):
    case = CASES[case_id]
    checked, result = run_in(case, ref_answer(case_id))
    assert result is None or result.state != "SUCCEEDED"
    if "fix_patch" in case:
        _, fixed = run_in(case, ref_answer(case_id), case["fix_patch"])
        assert fixed is not None and fixed.state == "SUCCEEDED"
    else:
        assert case["no_fix"]


@pytest.mark.parametrize("case_id", sorted(c for c in CASES if CASES[c]["class"] == "unsupported"))
def test_unsupported_has_an_argument_and_deferred_constructs_are_rejected(case_id):
    case = CASES[case_id]
    assert case["construct"] in lib.CONSTRUCTS and case["argument"]
    if "attempt_code" in case:
        checked = v1.check_answer(case, lib.reference(case_id))
        assert not checked.valid and case["attempt_code"] in checked.codes


@pytest.mark.parametrize("case_id", sorted(c for c in CASES if CASES[c]["class"] == "ambiguous"))
def test_ambiguous_every_reading_succeeds_and_they_differ(case_id):
    case = CASES[case_id]
    names = readings(case_id)
    assert len(names) >= 2 and case["slot"] in lib.SLOTS
    seen = set()
    for name in names:
        answer = lib.reference(name)
        _, result = run_in(case, answer)
        assert result is not None and result.state == "SUCCEEDED", name
        seen.add((answer["rob"], tuple(sorted(answer["bindings"].items()))))
    assert len(seen) == len(names)


def _swap(answer: dict, old: str, new: str) -> dict:
    answer["bindings"] = {k: (new if v == old else v) for k, v in answer["bindings"].items()}
    return answer


@pytest.mark.parametrize("case_id, answer, outcome", [
    ("f01_straddle_gate", other("reject", "infeasible"), "over_abstain"),
    ("d3_lane2_right_now", {**ref_answer("d3_lane2_right_now"),
                            "rob": lib.reference("d3_lane2_right_now")["rob"].replace("    pre { region_clear z; }\n", "")},
     "wrong_program"),
    ("f01_straddle_gate", {**ref_answer("f01_straddle_gate"), "rob": "robolang 1.3; task"}, "invalid"),
    ("i01_picker2_not_ready", other("reject", "infeasible", entities=["picker2"]), "correct_reject"),
    ("i09_no_morphine", other("reject", "infeasible"), "correct_reject"),
    ("i01_picker2_not_ready", other("reject", "unsupported", "other"), "reject_wrong_reason"),
    ("i01_picker2_not_ready", other("reject", "infeasible", entities=["tray_1"]), "reject_wrong_reason"),
    ("i01_picker2_not_ready", other("clarify", slot="robot", entities=["picker1"]), "clarify_instead"),
    ("i01_picker2_not_ready", ref_answer("i01_picker2_not_ready"), "faithful_program"),
    ("i01_picker2_not_ready", _swap(ref_answer("i01_picker2_not_ready"), "picker2", "picker1"), "silent_substitution"),
    ("i19_deadline_hot_zone", {**ref_answer("i19_deadline_hot_zone"),
                               "rob": lib.reference("i19_deadline_hot_zone")["rob"].replace("10 s", "45 s")},
     "silent_substitution"),
    ("u09_avoid_quay", other("reject", "unsupported", "path_constraint"), "correct_reject"),
    ("u09_avoid_quay", ref_answer("u09_avoid_quay"), "attempted_construct"),
    ("u09_avoid_quay", {**ref_answer("u09_avoid_quay"),
                        "rob": lib.reference("u09_avoid_quay")["rob"].replace("    @safety no_go_region z;\n", "")},
     "silent_substitution"),
    ("a01_which_vial", other("clarify", slot="object", entities=["vial_a2", "vial_a1"]), "correct_clarify"),
    ("a01_which_vial", other("clarify", slot="object", entities=["vial_a1"]), "clarify_wrong"),
    ("a13_keep_it_slow", other("clarify", slot="speed"), "correct_clarify"),
    ("a01_which_vial", other("reject", "infeasible"), "reject_instead"),
    ("a01_which_vial", {**ref_answer("a01_which_vial.1")}, "silent_guess"),
    ("a01_which_vial", None, "no_answer"),
])
def test_outcome_classes(case_id, answer, outcome):
    assert lib.classify(CASES[case_id], answer)["outcome"] == outcome


# -------- the runner, with a stand-in client --------

def _wire(answer: dict) -> str:
    return json.dumps({**answer, "bindings": [{"param": k, "id": v} for k, v in answer["bindings"].items()]})


class FakeClient:
    """Answers with the reference program, except an invalid first program for one case (to force a repair)."""

    def __init__(self):
        self.messages = self
        self.requests = []

    def create(self, **request):
        self.requests.append(request)
        text = request["messages"][0]["content"]
        case = next(c for c in CASES.values() if c["prompt"] in text)
        if case["id"] == "f01_straddle_gate" and len(request["messages"]) == 1:
            reply = {**ref_answer("f01_straddle_gate"), "rob": "robolang 1.3; task"}
        elif case["class"] == "ambiguous":
            reply = ref_answer(f"{case['id']}.1")
        elif case["class"] == "unsupported":
            reply = other("reject", "unsupported", case["construct"])
        else:
            reply = ref_answer(case["id"])
        return {"id": "msg", "model": request["model"], "stop_reason": "end_turn",
                "content": [{"type": "text", "text": _wire(reply)}],
                "usage": {"input_tokens": 1000, "output_tokens": 500, "cache_creation_input_tokens": 0,
                          "cache_read_input_tokens": 3000}}


def test_run_and_score_end_to_end(tmp_path):
    client = FakeClient()
    wanted = ["f01_straddle_gate", "i01_picker2_not_ready", "u09_avoid_quay", "a01_which_vial"]
    cases = {c: CASES[c] for c in wanted}
    text = lib.PROMPTS["full"].read_text(encoding="utf-8")
    spent = run.run_benchmark(client, ["claude-haiku-5-5"], cases, "medium", 1, tmp_path / "r", text, "full",
                              max_usd=5, workers=1, fallbacks=False, log=io.StringIO())
    assert len(client.requests) == 5 and spent > 0  # four answers, one repair round
    assert client.requests[0]["output_config"]["format"]["schema"] is lib.ANSWER_SCHEMA
    _, rows = score.load_rows(tmp_path / "r")
    assert {r["case_id"]: r["outcome"] for r in rows} == {
        "f01_straddle_gate": "correct", "i01_picker2_not_ready": "faithful_program",
        "u09_avoid_quay": "correct_reject", "a01_which_vial": "silent_guess"}
    assert "silent_guess" in score.score_dir(tmp_path / "r")


def test_the_cap_stops_new_requests(tmp_path):
    client = FakeClient()
    cases = {c: CASES[c] for c in ["f02_scan_fridge_vial", "f03_hold_tray_at_pass", "f04_crane_straddle_relay"]}
    run.run_benchmark(client, ["claude-opus-5-5"], cases, "medium", 1, tmp_path / "r",
                      lib.PROMPTS["full"].read_text(encoding="utf-8"), "full", max_usd=0.001, workers=1,
                      fallbacks=False, log=io.StringIO())
    assert len(client.requests) == 1


def test_estimate_without_yes_sends_nothing(tmp_path):
    out = io.StringIO()
    assert run.main(["--out", str(tmp_path)], client_factory=None, out=out, environ={}) == 2
    assert "Nothing was sent" in out.getvalue() and not any(tmp_path.iterdir())
