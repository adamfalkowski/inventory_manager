# Data Class
from dataclasses import dataclass, field

@dataclass()
class ColorData:
    color: str
    item_count: int
    # Lists are mutable so when a instance object modifies them, they actually change the default value
    # Need additional field function to make sure each instance gets its own version of the list;
    cx: list[float] = field(default_factory=list)
    cy: list[float] = field(default_factory=list)
    area: list[float] = field(default_factory=list)