# Specification: Backend Deployment to Hugging Face Spaces

## 1. Overview
The "AI-Native Textbook" features a RAG-based chatbot. To enable this on the public GitHub Pages site without costs, we will deploy the backend API to **Hugging Face Spaces** using their free Docker tier.

## 2. Requirements

### 2.1 Backend Hosting
- **Provider:** Hugging Face Spaces
- **Service Type:** Docker Space
- **Tier:** Free (CPU Basic)
- **Port:** 7860 (Standard for HF Spaces)
- **Environment Variables:**
  - `OPENAI_API_KEY`: Required
  - `GEMINI_API_KEY`: Required
  - `QDRANT_URL`: Required
  - `QDRANT_API_KEY`: Required

### 2.2 Frontend Integration
- **API URL:** The frontend chat component must target the production HF Space URL.
- **CORS:** The backend must allow requests from `https://muzamil-ai-dev.github.io`.

### 2.3 Infrastructure as Code
- A `Dockerfile` in the **root** directory will be used to define the build context, copying the `apps/api` code into the container.

## 3. Architecture

### Proposed (Production)
- Frontend: `https://Muzamil-Ai-Dev.github.io/Ai_book/`
- Backend: `https://<hf-username>-<space-name>.hf.space`
- Communication: HTTPS fetch with CORS validation

## 4. Implementation Steps
1.  **Root Dockerfile:** Create a `Dockerfile` in the project root to build `apps/api`.
2.  **Deployment:** User connects repository to Hugging Face Spaces.
3.  **Update Frontend:** Modify `apps/web/src/components/Chat/index.tsx` with the new URL.
