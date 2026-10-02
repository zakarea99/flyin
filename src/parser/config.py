from pydantic import BaseModel, Field, ValidationError

class config(BaseModel):
    nb_drones: int = Field(gt=0)