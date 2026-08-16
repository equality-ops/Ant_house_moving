from typing import TypeVar

_T = TypeVar("_T")

def const(value: _T) -> _T: ...
