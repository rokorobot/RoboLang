"""
The Core 0.1 schema files and a validator for the JSON Schema 2020-12 keywords they use.

The package has no runtime dependencies (decision R12), so this module interprets exactly
the keywords that spec/core/*.schema.json contain: `$ref` (local), `type`, `const`, `enum`,
`pattern`, `required`, `properties`, `additionalProperties: false`, `items`, `minItems`,
`maxItems`, `uniqueItems`, `minimum`, `maximum`, `minLength`, `maxLength`, `oneOf`, `allOf`, `if`/`then`.
A schema using any other keyword is refused at load time rather than half-checked.
The tests compare this validator with the `jsonschema` library on every fixture.
"""

from __future__ import annotations
import json
import re
from dataclasses import dataclass
from functools import lru_cache
from importlib import resources

MESSAGE_SCHEMA = "message-0.1.schema.json"
MESSAGE_SCHEMA_02 = "message-0.2.schema.json"
MESSAGE_SCHEMAS = {"0.1": MESSAGE_SCHEMA, "0.2": MESSAGE_SCHEMA_02}
DESCRIPTOR_SCHEMA = "capability-descriptor-0.1.schema.json"

_ANNOTATIONS = {"$schema", "$id", "$defs", "title", "description"}
_KEYWORDS = {
    "$ref", "type", "const", "enum", "pattern", "required", "properties", "additionalProperties", "items",
    "minItems", "maxItems", "uniqueItems", "minimum", "maximum", "minLength", "maxLength", "oneOf", "allOf", "if",
    "then",
}


@dataclass(frozen=True)
class SchemaError:
    """One failure: the keyword that failed and a JSON Pointer to the instance location."""

    keyword: str
    pointer: str


def _check_keywords(node) -> None:
    """Refuse a schema that uses a keyword this validator does not implement."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key in ("properties", "$defs"):
                for sub in value.values():
                    _check_keywords(sub)
            elif key in ("const", "enum") or key in _ANNOTATIONS - {"$defs"}:
                continue
            elif key not in _KEYWORDS:
                raise ValueError(f"unsupported schema keyword {key!r}")
            else:
                _check_keywords(value)
    elif isinstance(node, list):
        for item in node:
            _check_keywords(item)


@lru_cache(maxsize=None)
def load_schema(name: str) -> dict:
    """A Core schema shipped with the package (a copy of spec/core/<name>)."""
    text = resources.files("robolang.core").joinpath("schemas", name).read_text(encoding="utf-8")
    schema = json.loads(text)
    _check_keywords(schema)
    return schema


_TYPES = {
    "object": lambda v: isinstance(v, dict),
    "array": lambda v: isinstance(v, list),
    "string": lambda v: isinstance(v, str),
    "integer": lambda v: isinstance(v, int) and not isinstance(v, bool),
    "boolean": lambda v: isinstance(v, bool),
    "null": lambda v: v is None,
}


def _pointer(path: tuple) -> str:
    return "".join("/" + str(p).replace("~", "~0").replace("/", "~1") for p in path)


def _equal(a, b) -> bool:
    """JSON equality: 1 and True differ."""
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    return a == b


class _Validator:
    def __init__(self, root: dict):
        self._root = root

    def _resolve(self, ref: str) -> dict:
        if not ref.startswith("#/"):
            raise ValueError(f"only local references are supported: {ref}")
        node = self._root
        for token in ref[2:].split("/"):
            node = node[token.replace("~1", "/").replace("~0", "~")]
        return node

    def errors(self, value, schema: dict, path: tuple = ()) -> list[SchemaError]:
        out: list[SchemaError] = []
        if "$ref" in schema:
            out += self.errors(value, self._resolve(schema["$ref"]), path)
        if "type" in schema and not _TYPES[schema["type"]](value):
            return out + [SchemaError("type", _pointer(path))]
        if "const" in schema and not _equal(value, schema["const"]):
            out.append(SchemaError("const", _pointer(path)))
        if "enum" in schema and not any(_equal(value, option) for option in schema["enum"]):
            out.append(SchemaError("enum", _pointer(path)))
        if isinstance(value, str):
            if "pattern" in schema and re.search(schema["pattern"], value) is None:
                out.append(SchemaError("pattern", _pointer(path)))
            if "minLength" in schema and len(value) < schema["minLength"]:
                out.append(SchemaError("minLength", _pointer(path)))
            if "maxLength" in schema and len(value) > schema["maxLength"]:
                out.append(SchemaError("maxLength", _pointer(path)))
        if isinstance(value, int) and not isinstance(value, bool):
            if "minimum" in schema and value < schema["minimum"]:
                out.append(SchemaError("minimum", _pointer(path)))
            if "maximum" in schema and value > schema["maximum"]:
                out.append(SchemaError("maximum", _pointer(path)))
        if isinstance(value, dict):
            properties = schema.get("properties", {})
            for name in schema.get("required", []):
                if name not in value:
                    out.append(SchemaError("required", _pointer(path + (name,))))
            for name, item in value.items():
                if name in properties:
                    out += self.errors(item, properties[name], path + (name,))
                elif schema.get("additionalProperties") is False:
                    out.append(SchemaError("additionalProperties", _pointer(path + (name,))))
        if isinstance(value, list):
            if "minItems" in schema and len(value) < schema["minItems"]:
                out.append(SchemaError("minItems", _pointer(path)))
            if "maxItems" in schema and len(value) > schema["maxItems"]:
                out.append(SchemaError("maxItems", _pointer(path)))
            if schema.get("uniqueItems"):
                for i, item in enumerate(value):
                    if any(_equal(item, other) for other in value[:i]):
                        out.append(SchemaError("uniqueItems", _pointer(path)))
                        break
            if "items" in schema:
                for i, item in enumerate(value):
                    out += self.errors(item, schema["items"], path + (i,))
        if "oneOf" in schema:
            matches = sum(not self.errors(value, branch, path) for branch in schema["oneOf"])
            if matches != 1:
                out.append(SchemaError("oneOf", _pointer(path)))
        for branch in schema.get("allOf", []):
            out += self.errors(value, branch, path)
        if "if" in schema and not self.errors(value, schema["if"], path) and "then" in schema:
            out += self.errors(value, schema["then"], path)
        return out


def validate(document, schema_name: str) -> list[SchemaError]:
    """Every schema failure of `document`, in a deterministic order. Empty when valid."""
    schema = load_schema(schema_name)
    return _Validator(schema).errors(document, schema)
