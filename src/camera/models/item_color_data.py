# Data Class
from dataclasses import dataclass

@dataclass()
class ColorData:
    color: str
    item_count: int
    cx: float
    cy: float
