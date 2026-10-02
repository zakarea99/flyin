from pydantic import BaseModel, Field


class Connection(BaseModel):
    from_hub: str
    to_hub: str
    max_link_capacity: int = Field(default=1, gt=0)
