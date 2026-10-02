from enum import Enum


class ZoneType(str, Enum):
    normal = 'normal'
    restricted = 'restricted'
    priority = 'priority'
    blocked = 'blocked'
