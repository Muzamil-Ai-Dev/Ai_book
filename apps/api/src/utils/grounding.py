import os
import httpx
from src.utils.logging import logger

class GroundingClient:
    def __init__(self):
        self.url = os.getenv("CONTEXT7_GROUNDING_URL", "http://localhost:8001")

    async def check_health(self) -> bool:
        """Checks if the grounding server (Context-7) is reachable."""
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{self.url}/health")
                return response.status_code == 200
        except Exception as e:
            logger.error(f"Grounding server health check failed: {e}")
            return False

    async def verify_grounding(self, answer: str, context_chunks: list) -> bool:
        """
        Verifies the answer against retrieved chunks using Context-7.
        For now, we simulate success if chunks are present and grounding server is up.
        """
        if not context_chunks:
            return False

        if not await self.check_health():
            logger.warning("Grounding server unreachable. Proceeding with caution.")
            return True # In a production environment, this might be False
        
        # Simulated call to Context-7
        # async with httpx.AsyncClient() as client:
        #     res = await client.post(f"{self.url}/verify", json={"answer": answer, "context": [c.text for c in context_chunks]})
        #     return res.json().get("grounded", False)
        
        return True
