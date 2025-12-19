import os
import cohere
from src.utils.vector_db import VectorDBClient
from src.models.content import ContentChunk, SourceMetadata

class RetrievalService:
    def __init__(self):
        self.vector_db = VectorDBClient()
        api_key = os.getenv("COHERE_API_KEY")
        self.co = cohere.Client(api_key) if api_key else None

    def retrieve_relevant_chunks(self, query: str, limit: int = 5, threshold: float = 0.5):
        """Retrieves chunks from vector DB based on query embedding."""
        response = self.co.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_vector = response.embeddings[0]
        
        results = self.vector_db.search(vector=query_vector, limit=limit, threshold=threshold)
        
        chunks = []
        for res in results:
            payload = res.payload
            chunks.append(ContentChunk(
                id=str(res.id),
                text=payload["text"],
                metadata=SourceMetadata(**payload["metadata"]),
                order=payload["order"]
            ))
            
        return chunks
