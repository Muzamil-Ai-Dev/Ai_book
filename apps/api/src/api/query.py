from fastapi import APIRouter
from pydantic import BaseModel
from typing import Optional
from src.services.chat_service import ChatService

router = APIRouter()
chat_service = ChatService()

class QueryRequest(BaseModel):
    question: str
    session_id: Optional[str] = None

@router.post("/query")
async def query_chatbot(request: QueryRequest):
    session_id = request.session_id or "default"
    result = await chat_service.generate_response(request.question, session_id=session_id)
    return result
