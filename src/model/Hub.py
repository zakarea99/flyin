from pydantic import BaseModel, Field, field_validator
from model.ZoneType import ZoneType


class Hub(BaseModel):
    name: str
    x: int
    y: int
    zone_type: ZoneType = Field(default=ZoneType.normal)
    max_drones: int = Field(default=1, gt=0)
    color: str = "none"
    is_start: bool = Field(default=False)
    is_end: bool = Field(default=False)

    @field_validator('name')
    @staticmethod
    def validate_name(value: str) -> str:
        if "-" in value or " " in value:
            raise ValueError(f"Invalid name '{value}'")
        return value
