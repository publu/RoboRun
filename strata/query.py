"""Filter expressions — one language for blob labels and vector attributes.

JSON-friendly AST in the turbopuffer style:

    ["floor", "Eq", 3]
    ["label", "In", ["pallet", "crate"]]
    ["score", "Gte", 0.8]
    ["And", [["floor", "Eq", 3], ["label", "Eq", "pallet"]]]
    ["Or",  [ ... ]]
    ["Not",   [ ... ] ]
    ["name", "Glob", "cam_*"]

Operators: Eq NotEq Gt Gte Lt Lte In NotIn Glob Contains. Missing fields never
match a positive predicate (and always match `NotEq`/`NotIn`). The same
`matches()` evaluates a row of attributes; `as_predicate()` compiles a filter to
a fast closure for hot scan loops.
"""
from __future__ import annotations

import fnmatch
from typing import Callable, Optional

_LOGICAL = {"And", "Or", "Not"}


def _cmp(op: str, have, want) -> bool:
    if op == "Eq":
        return have == want
    if op == "NotEq":
        return have != want
    if have is None:
        # ordered / membership comparisons against a missing field don't match
        return False
    try:
        if op == "Gt":
            return have > want
        if op == "Gte":
            return have >= want
        if op == "Lt":
            return have < want
        if op == "Lte":
            return have <= want
    except TypeError:
        return False
    if op == "In":
        return have in want
    if op == "NotIn":
        return have not in want
    if op == "Glob":
        return isinstance(have, str) and fnmatch.fnmatchcase(have, str(want))
    if op == "Contains":
        try:
            return want in have
        except TypeError:
            return False
    raise ValueError(f"unknown operator: {op!r}")


def matches(flt: Optional[list], attrs: dict) -> bool:
    if not flt:
        return True
    head = flt[0]
    if head in _LOGICAL:
        if head == "Not":
            sub = flt[1]
            return not matches(sub, attrs)
        subs = flt[1]
        if head == "And":
            return all(matches(s, attrs) for s in subs)
        return any(matches(s, attrs) for s in subs)   # Or
    # comparison: [field, op, value]
    field, op, want = flt[0], flt[1], (flt[2] if len(flt) > 2 else None)
    have = attrs.get(field)
    if op == "NotEq" and field not in attrs:
        return True
    if op == "NotIn" and field not in attrs:
        return True
    return _cmp(op, have, want)


def as_predicate(flt: Optional[list]) -> Callable[[dict], bool]:
    if not flt:
        return lambda _a: True
    return lambda a: matches(flt, a)


def referenced_fields(flt: Optional[list]) -> set[str]:
    """Fields a filter touches — used to keep only needed attribute columns hot."""
    if not flt:
        return set()
    head = flt[0]
    if head == "Not":
        return referenced_fields(flt[1])
    if head in ("And", "Or"):
        out: set[str] = set()
        for s in flt[1]:
            out |= referenced_fields(s)
        return out
    return {flt[0]}
