# Tasks: Hugging Face Deployment

## 1. Backend Configuration
- [x] **Update CORS settings**: Modify `apps/api/src/main.py` to allow `https://muzamil-ai-dev.github.io`. <!-- id: 1 -->

## 2. Infrastructure
- [ ] **Create Root `Dockerfile`**: Define the Docker build for HF Spaces. <!-- id: 2 -->
    - Base: `python:3.11-slim`
    - Context: Copy `apps/api` to container.
    - Port: `7860`.

    - Build Command: `pip install -r apps/api/requirements.txt`
    - Start Command: `cd apps/api && uvicorn src.main:app --host 0.0.0.0 --port $PORT`
    - Env Vars: `GEMINI_API_KEY`, `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `PYTHON_VERSION`.

## 3. Frontend Update (Pending Deployment)
- [ ] **Update API Endpoint**: Change the production API URL in `apps/web/src/components/Chat/index.tsx`. <!-- id: 3 -->
    - *Note:* This task requires the deployed HF Space URL.
