# AI-Native Textbook RAG API

This is the backend for the AI-Native Textbook RAG Chatbot. It provides endpoints for ingesting textbook content and querying the grounded chatbot.

## Core Features
- **Grounded RAG**: Answers are strictly derived from the textbook Markdown content.
- **Hard Grounding**: Uses Context-7 (MCP) logic to prevent hallucinations.
- **Conversational Learning**: Supports session-based multi-turn dialogue.
- **Source Transparency**: Every answer includes metadata for the original textbook sections.

## Tech Stack
- FastAPI (Backend)
- OpenAI ChatKit SDK (Orchestration)
- Gemini (LLM Generation)
- Qdrant (Vector Database)

## Setup

1. **Environment Variables**
   Configure `.env` in this directory:
   ```env
   OPENAI_API_KEY=...
   GEMINI_API_KEY=...
   QDRANT_URL=...
   QDRANT_API_KEY=...
   CONTEXT7_GROUNDING_URL=http://localhost:8001
   ```

2. **Ingest Content**
   ```bash
   curl -X POST http://localhost:8000/ingest
   ```

3. **Query Chatbot**
   ```bash
   curl -X POST http://localhost:8000/query \
        -H "Content-Type: application/json" \
        -d '{"question": "What is a ROS 2 node?"}'
   ```

