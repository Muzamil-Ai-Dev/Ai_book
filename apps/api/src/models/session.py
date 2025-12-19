from pydantic import BaseModel
from typing import List, Optional

class ChatTurn(BaseModel):
    role: str # user, assistant
    content: str

class ConversationSession(BaseModel):
    session_id: str
    history: List[ChatTurn] = []
