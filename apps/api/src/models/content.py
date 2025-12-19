from pydantic import BaseModel, Field
from typing import List, Optional

class SourceMetadata(BaseModel):
    module: str
    chapter: str
    heading: str
    path: str

class ContentChunk(BaseModel):
    id: str
    text: str
    metadata: SourceMetadata
    order: int
