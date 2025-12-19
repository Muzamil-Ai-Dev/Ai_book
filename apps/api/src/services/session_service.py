from typing import Dict
from src.models.session import ConversationSession

class SessionStore:
    def __init__(self):
        self.sessions: Dict[str, ConversationSession] = {}

    def get_or_create_session(self, session_id: str) -> ConversationSession:
        if session_id not in self.sessions:
            self.sessions[session_id] = ConversationSession(session_id=session_id)
        return self.sessions[session_id]

    def add_turn(self, session_id: str, role: str, content: str):
        session = self.get_or_create_session(session_id)
        from src.models.session import ChatTurn
        session.history.append(ChatTurn(role=role, content=content))
        # Keep history manageable
        if len(session.history) > 10:
            session.history = session.history[-10:]

session_store = SessionStore()
