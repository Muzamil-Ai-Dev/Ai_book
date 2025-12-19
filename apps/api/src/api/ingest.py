from fastapi import APIRouter
from src.services.ingestion import IngestionService

router = APIRouter()
ingestion_service = IngestionService()

@router.post("/ingest")
async def ingest_content():
    try:
        count = ingestion_service.ingest_all()
        return {"status": "success", "chunks_processed": count}
    except Exception as e:
        return {"status": "error", "message": str(e)}
