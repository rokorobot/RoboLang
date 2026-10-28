"""
The participant view of `robolang-sim-stdio` 0.1 (spec/interop-gate-2-v0.1.md 6.4).

Only what the participant would hold as its own configuration, plus the public material it
needs. Never other participants' private configuration, the planner's goal, the authority's
decision table, durations, faults, the case ID or the expected results.
"""

from __future__ import annotations

from ..arch02.scenario import counterparts, trusted_key_refs

PARAMETERS = ("epoch", "message_ttl", "max_message_lifetime", "clock_tolerance", "max_hold",
              "offer_lease_end", "reservation_end")


def build_view(scenario: dict, participant: str) -> dict:
    """The view of `participant` in `scenario` (after the case's overrides)."""
    records = {entry["id"]: entry for entry in scenario["participants"]}
    others = counterparts(scenario, participant)
    own_key = f"{participant}#k1"
    location = next(f["value"] for f in scenario["initial_facts"]
                    if f["subject"] == participant and f["property"] == "located_in")
    return {
        "participant": participant,
        "key": {"ref": own_key, **scenario["keys"][own_key]},
        "descriptor": scenario["descriptors"].get(participant),
        "standing_authorization": scenario["standing_authorizations"].get(participant),
        "policy": scenario["policies"][participant],
        # Own sensors with the characteristics an observation states (Core 5.6); see finding E2.
        "sensors": [{k: v for k, v in s.items() if k != "participant"}
                    for s in scenario["sensors"] if s["participant"] == participant],
        "location": location,
        "parameters": {name: scenario["parameters"][name] for name in PARAMETERS},
        "conversation_id": scenario["conversation_id"],
        "participants": [records[pid] for pid in [participant, *others]],
        "places": scenario["places"],
        "arbiters": scenario["arbiters"],
        "authority": scenario["authorities"][participant],
        # Its trust context besides its own key (reference simulation 5.3, decision record 0013).
        "verification_keys": {ref: scenario["keys"][ref] for ref in trusted_key_refs(scenario, participant)[1:]},
    }
