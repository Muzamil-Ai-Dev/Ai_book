from pydantic import BaseModel
from typing import List, Optional
from src.models.content import SourceMetadata

class ChatResponse(BaseModel):
    answer: str
    sources: List[SourceMetadata]
    refusal_reason: Optional[str] = None
