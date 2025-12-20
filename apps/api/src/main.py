import os
from dotenv import load_dotenv

# Explicitly load .env from the app directory at the very start
env_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.env"))
load_dotenv(dotenv_path=env_path, override=True)

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.utils.logging import logger
from src.api.ingest import router as ingestion_router
from src.api.query import router as query_router

logger.info(f"DEBUG: Looking for .env at {env_path}")
logger.info(f"DEBUG: .env exists: {os.path.exists(env_path)}")
logger.info(f"DEBUG: COHERE_API_KEY present: {bool(os.getenv('COHERE_API_KEY'))}")

app = FastAPI(title="AI-Native Textbook RAG API")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000", 
        "http://127.0.0.1:3000",
        "https://muzamil-ai-dev.github.io"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(ingestion_router)
app.include_router(query_router)

@app.get("/")
async def root():
    return {"message": "AI-Native Textbook RAG API is running"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)