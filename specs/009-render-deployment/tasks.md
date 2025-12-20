# Tasks: Render Deployment

## 1. Backend Configuration
- [ ] **Update CORS settings**: Modify `apps/api/src/main.py` to allow `https://muzamil-ai-dev.github.io`. <!-- id: 1 -->

## 2. Infrastructure
- [ ] **Create `render.yaml`**: Define the Render service blueprint. <!-- id: 2 -->
    - Service Name: `ai-book-api`
    - Environment: Python
    - Build Command: `pip install -r apps/api/requirements.txt`
    - Start Command: `cd apps/api && uvicorn src.main:app --host 0.0.0.0 --port $PORT`
    - Env Vars: `OPENAI_API_KEY`, `GEMINI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `PYTHON_VERSION`.

## 3. Frontend Update (Pending Deployment)
- [ ] **Update API Endpoint**: Change the production API URL in `apps/web/src/components/Chat/index.tsx`. <!-- id: 3 -->
    - *Note:* This task requires the deployed Render service URL.
