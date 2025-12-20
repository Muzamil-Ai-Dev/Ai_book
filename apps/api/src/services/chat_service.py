import os
from openai import OpenAI
from src.services.retrieval import RetrievalService
from src.utils.grounding import GroundingClient
from src.utils.logging import logger

from src.services.session_service import session_store
from src.models.session import ChatTurn

class ChatService:
    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.grounding_client = GroundingClient()
        
        # Configure Groq (OpenAI Compatible)
        # Using the API key from GEMINI_API_KEY env var as requested by user
        self.client = OpenAI(
            api_key=os.getenv("GEMINI_API_KEY"),
            base_url="https://api.groq.com/openai/v1"
        )
        self.system_prompt = (
            "You are an AI assistant for the AI-Native Textbook for Physical AI & Humanoid Robotics. "
            "Your goal is to answer questions strictly grounded in the provided textbook content. "
            "1. For technical questions, ONLY answer from the book content provided in the context. "
            "2. For greetings or small talk (like 'hello', 'who are you?'), respond politely and briefly as a tutor, then encourage the user to ask questions about the book. "
            "3. If the user asks for technical information that is missing from the context, politely refuse and state that the information is not in the book. "
            "4. Do not use external technical knowledge or speculate. "
            "5. Be concise and educational."
        )

    async def generate_response(self, question: str, session_id: str = "default"):
        # 0. Get session history
        session = session_store.get_or_create_session(session_id)
        
        # 1. Retrieve with threshold
        chunks = self.retrieval_service.retrieve_relevant_chunks(question, threshold=0.5)
        
        # If no chunks, we still call the LLM to allow for greetings/refusals
        context_text = "\n\n".join([f"Source: {c.metadata.path}\nContent: {c.text}" for c in chunks]) if chunks else "No relevant textbook content found."
        
        # 3. Generate with Groq (OpenAI Compatible)
        try:
            # Format history for OpenAI format
            messages = [{"role": "system", "content": self.system_prompt}]
            
            for turn in session.history:
                messages.append(
                    {"role": "user" if turn.role == "user" else "assistant", "content": turn.content}
                )

            # Add current context and prompt
            prompt = f"TEXTBOOK CONTEXT:\n{context_text}\n\nUSER QUESTION: {question}"
            messages.append({"role": "user", "content": prompt})

            # Create completion
            response = self.client.chat.completions.create(
                model="openai/gpt-oss-20b",
                messages=messages,
                temperature=0.0 # Keep it deterministic for RAG
            )
            
            answer = response.choices[0].message.content
            
            # 4. Final Grounding Verification (only if chunks were found)
            if chunks:
                is_grounded = await self.grounding_client.verify_grounding(answer, chunks)
                if not is_grounded:
                    return {
                        "answer": "I am sorry, but I cannot provide a grounded answer for that question based on the textbook.", 
                        "sources": [],
                        "refusal_reason": "Grounding verification failed."
                    }

            # 5. Save to history
            session_store.add_turn(session_id, "user", question)
            session_store.add_turn(session_id, "assistant", answer)

            sources = [c.metadata.model_dump() for c in chunks] if chunks else []
            
            return {"answer": answer, "sources": sources, "session_id": session_id}
        except Exception as e:
            logger.error(f"Generation failed: {e}")
            return {"answer": f"An error occurred while generating the response: {str(e)}", "sources": []}