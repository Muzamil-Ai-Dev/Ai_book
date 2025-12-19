# Quickstart: Grounded RAG Chatbot

This guide helps you set up and run the RAG chatbot backend.

## Prerequisites
- Python 3.11+
- API Keys:
  - `OPENAI_API_KEY` (for ChatKit and Embeddings)
  - `GEMINI_API_KEY` (for generation)
  - `QDRANT_API_KEY` & `QDRANT_URL` (for vector storage)

## Setup

1. **Clone and Install**
   ```bash
   cd apps/api
   pip install -r requirements.txt
   ```

2. **Environment Configuration**
   Create a `.env` file in `apps/api/`:
   ```env
   OPENAI_API_KEY=your_key
   GEMINI_API_KEY=your_key
   QDRANT_URL=your_url
   QDRANT_API_KEY=your_key
   ```

3. **Ingest Content**
   Run the ingestion script to process the textbook modules:
   ```bash
   python -m src.scripts.ingest
   # OR use the endpoint
   curl -X POST http://localhost:8000/ingest
   ```

4. **Run the API**
   ```bash
   uvicorn src.main:app --reload
   ```

## Example Query
```bash
curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d '{"question": "What is a VLA model?"}'
```

## Troubleshooting
- **No answer/Refusal**: Check if the content has been ingested correctly and if the question is covered in `content/modules/`.
- **Grounding Errors**: Ensure the Context-7 MCP server is reachable and configured.
