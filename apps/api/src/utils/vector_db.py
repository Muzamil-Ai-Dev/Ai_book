import os
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.http import models

class VectorDBClient:
    def __init__(self):
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.client = QdrantClient(url=self.url, api_key=self.api_key)
        self.collection_name = "textbook_content"

    def ensure_collection(self, vector_size: int = 1536):
        """Ensures the collection exists in Qdrant."""
        collections = self.client.get_collections().collections
        exists = any(c.name == self.collection_name for c in collections)
        
        if not exists:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
            )

    def upsert_chunks(self, points: List[models.PointStruct]):
        """Upserts chunks into the vector database."""
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search(self, vector: List[float], limit: int = 5, threshold: float = 0.5):
        """Searches for relevant chunks based on vector similarity with score threshold."""
        response = self.client.query_points(
            collection_name=self.collection_name,
            query=vector,
            limit=limit,
            score_threshold=threshold
        )
        return response.points
