# Specification: Backend Deployment to Render

## 1. Overview
The "AI-Native Textbook" features a RAG-based chatbot that currently runs locally. To enable this feature on the public GitHub Pages deployment, the backend API (`apps/api`) must be hosted on a public server. We will use Render.com for this purpose.

## 2. Requirements

### 2.1 Backend Hosting
- **Provider:** Render
- **Service Type:** Web Service
- **Runtime:** Python 3
- **Build Command:** `pip install -r apps/api/requirements.txt`
- **Start Command:** `cd apps/api && uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- **Environment Variables:**
  - `OPENAI_API_KEY`: Required
  - `GEMINI_API_KEY`: Required
  - `QDRANT_URL`: Required
  - `QDRANT_API_KEY`: Required
  - `PYTHON_VERSION`: 3.11.9 (or compatible)

### 2.2 Frontend Integration
- **API URL:** The frontend chat component must target the production Render URL when not in development mode.
- **CORS:** The backend must allow requests from the GitHub Pages origin: `https://Muzamil-Ai-Dev.github.io`.

### 2.3 Infrastructure as Code
- A `render.yaml` file should be created in the root to define the service configuration, allowing for "Blueprint" deployments.

## 3. Architecture

### Current (Local)
- Frontend: `http://localhost:3000`
- Backend: `http://localhost:8000`
- Communication: Direct HTTP fetch

### Proposed (Production)
- Frontend: `https://Muzamil-Ai-Dev.github.io/Ai_book/`
- Backend: `https://ai-book-api.onrender.com` (Example URL)
- Communication: HTTPS fetch with CORS validation

## 4. Implementation Steps
1.  **Configure CORS:** Update `apps/api/src/main.py` to allow the GitHub Pages origin.
2.  **Create Render Blueprint:** Add `render.yaml` to the project root.
3.  **Update Frontend:** Modify `apps/web/src/components/Chat/index.tsx` to use the configured production URL.
4.  **Deployment:** User connects repository to Render and deploys.
