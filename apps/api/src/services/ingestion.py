import os
from glob import glob
from src.utils.chunking import chunk_markdown
from src.utils.vector_db import VectorDBClient
from src.utils.logging import logger
from qdrant_client.http import models as qdrant_models
import cohere

class IngestionService:
    def __init__(self):
        self.vector_db = VectorDBClient()
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            logger.error("COHERE_API_KEY not found in environment variables")
        self.co = cohere.Client(api_key) if api_key else None

    def ingest_all(self):
        """Finds all markdown files and ingests them."""
        # Adjust path to absolute for reliability
        root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
        content_path = os.path.join(root_dir, "apps/web/modules")
        
        try:
            logger.info(f"Starting ingestion from {content_path}")
            md_files = glob(f"{content_path}/**/*.md", recursive=True)
            logger.info(f"Found {len(md_files)} markdown files")
            
            total_chunks = 0
            for md_file in md_files:
                logger.info(f"Processing {md_file}")
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                # Extract module and chapter from path
                rel_path = os.path.relpath(md_file, content_path)
                parts = rel_path.split(os.sep)
                module = parts[0] if len(parts) > 1 else "root"
                chapter = parts[-1]
                
                chunks = chunk_markdown(content, rel_path, module, chapter)
                logger.info(f"Chunked {md_file} into {len(chunks)} chunks")
                
                if not chunks:
                    continue
                    
                # Generate embeddings in batches for Cohere
                texts = [chunk.text for chunk in chunks]
                response = self.co.embed(
                    texts=texts,
                    model="embed-english-v3.0",
                    input_type="search_document"
                )
                embeddings = response.embeddings
                
                points = []
                for i, chunk in enumerate(chunks):
                    points.append(qdrant_models.PointStruct(
                        id=chunk.id,
                        vector=embeddings[i],
                        payload={
                            "text": chunk.text,
                            "metadata": chunk.metadata.model_dump(),
                            "order": chunk.order
                        }
                    ))
                
                logger.info(f"Upserting {len(points)} points to vector DB")
                self.vector_db.ensure_collection(vector_size=len(points[0].vector))
                self.vector_db.upsert_chunks(points)
                total_chunks += len(points)
                logger.info(f"Successfully upserted chunks for {md_file}")
                
            logger.info(f"Ingestion complete. Total chunks: {total_chunks}")
            return total_chunks
        except Exception as e:
            import traceback
            logger.error(f"Ingestion failed: {e}")
            logger.error(traceback.format_exc())
            raise e
