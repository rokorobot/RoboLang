"""ed25519 signatures for Core, the optional extra robolang[ed25519] (decision 0038). Simulation only."""

from __future__ import annotations
import base64
import dataclasses
import json

import pytest

from robolang import core
from robolang.core import ED25519, Key, check_message, receiver, signatures
from test_core import CORE_FIXTURES, KEYS, context

pytest.importorskip("cryptography")

# RFC 8032 section 7.1, test 2: a one-byte message.
RFC_SEED = bytes.fromhex("4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb")
RFC_PUBLIC = bytes.fromhex("3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c")
RFC_SIGNATURE = bytes.fromhex(
    "92a009a9f0d4cab8720e820b5f642540a2b27b5416503f8fb3762223ebdb69da"
    "085ac1e43e15996e458f3613d0f11d8c387b2eaeb4302aeeb00d291612bb0c00"
)


def b64url(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode()


def test_rfc_8032_vector():
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    assert Ed25519PrivateKey.from_private_bytes(RFC_SEED).sign(b"\x72") == RFC_SIGNATURE


def test_round_trip_tampering_and_wrong_key():
    seed, public = core.generate_ed25519()
    key = Key("rl:robot:arm-02", ED25519, public_key=public)
    signed = core.sign_ed25519({"a": 1, "b": "x"}, "rl:robot:arm-02#k2", seed)
    value = signed["signature"]["value"]
    assert len(value) == 86 and "=" not in value
    assert core.verify_ed25519(signed, key)
    assert not core.verify_ed25519({**signed, "b": "y"}, key)
    assert not core.verify_ed25519(signed, Key("rl:robot:arm-02", ED25519, public_key=RFC_PUBLIC))
    assert not core.verify_ed25519(signed, Key("rl:robot:arm-02", ED25519, public_key=b"short"))


@pytest.mark.parametrize("bad", ["", "A" * 85, "A" * 86 + "==", "!" * 86])
def test_malformed_values_fail(bad):
    seed, public = core.generate_ed25519()
    signed = core.sign_ed25519({"a": 1}, "k", seed)
    signed["signature"]["value"] = bad
    assert not core.verify_ed25519(signed, Key("p", ED25519, public_key=public))


def test_a_second_encoding_of_the_same_bytes_fails():
    seed, public = core.generate_ed25519()
    signed = core.sign_ed25519({"a": 1}, "k", seed)
    value = signed["signature"]["value"]
    last = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_"
    flipped = last[last.index(value[-1]) ^ 1]  # changes only the 2 padding bits of the last character
    signed["signature"]["value"] = value[:-1] + flipped
    assert not core.verify_ed25519(signed, Key("p", ED25519, public_key=public))


def ed25519_message():
    """A valid fixture message re-signed by its sender with a fresh ed25519 key."""
    document = json.loads((CORE_FIXTURES / "valid" / "assessment.json").read_bytes())
    sender_ref = document["signature"]["key"]
    seed, public = core.generate_ed25519()
    ref = sender_ref.split("#")[0] + "#ed"
    keys = {**KEYS, ref: Key(KEYS[sender_ref].participant, ED25519, public_key=public)}
    return core.sign_ed25519(document, ref, seed), keys


def test_default_receiver_still_rejects_ed25519():
    signed, keys = ed25519_message()
    ctx = dataclasses.replace(context(signed["recipient"]), keys=keys)
    assert check_message(json.dumps(signed).encode(), ctx).code == "C204"


def test_receiver_that_lists_ed25519_accepts_it_and_catches_tampering():
    signed, keys = ed25519_message()
    ctx = dataclasses.replace(context(signed["recipient"]), keys=keys,
                              algorithms=frozenset({core.SIM_HMAC, ED25519}))
    assert check_message(json.dumps(signed).encode(), ctx).accepted
    tampered = {**signed, "signature": {**signed["signature"], "value": b64url(bytes(64))}}
    assert check_message(json.dumps(tampered).encode(), ctx).code == "C202"
    hmac_key_ref = next(r for r, k in KEYS.items() if k.participant == keys[signed["signature"]["key"]].participant)
    wrong_alg_key = {**signed, "signature": {**signed["signature"], "key": hmac_key_ref}}
    assert check_message(json.dumps(wrong_alg_key).encode(), ctx).code == "C203"


def test_physical_mode_still_rejects_ed25519():
    signed, keys = ed25519_message()
    ctx = dataclasses.replace(context(signed["recipient"], mode="physical"), keys=keys,
                              algorithms=frozenset({ED25519}))
    assert check_message(json.dumps(signed).encode(), ctx).code == "C204"


def test_without_cryptography_ed25519_is_not_accepted(monkeypatch):
    signed, keys = ed25519_message()
    monkeypatch.setattr(receiver, "ed25519_available", lambda: False)
    ctx = dataclasses.replace(context(signed["recipient"]), keys=keys, algorithms=frozenset({ED25519}))
    assert check_message(json.dumps(signed).encode(), ctx).code == "C204"


def test_missing_extra_names_the_install_command(monkeypatch):
    import builtins
    real = builtins.__import__

    def no_crypto(name, *args, **kwargs):
        if name.startswith("cryptography"):
            raise ImportError(name)
        return real(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_crypto)
    assert not signatures.ed25519_available()
    with pytest.raises(RuntimeError, match=r"robolang\[ed25519\]"):
        signatures.sign_ed25519({"a": 1}, "k", RFC_SEED)
    with pytest.raises(RuntimeError, match=r"robolang\[ed25519\]"):
        signatures.generate_ed25519()
