# Plan: Hugging Face Spaces Deployment

## 1. Prepare Backend for Production
- [x] Update `apps/api/src/main.py` to include `https://Muzamil-Ai-Dev.github.io` in CORS `allow_origins` (Done previously).
- [ ] Verify `apps/api/requirements.txt`.

## 2. Infrastructure as Code
- [ ] Create `Dockerfile` in the **root** directory.
    - Base: `python:3.11-slim`
    - Copy `apps/api/requirements.txt` to `/code/requirements.txt`.
    - Install dependencies.
    - Copy `apps/api` to `/code/apps/api`.
    - Workdir: `/code/apps/api`.
    - Expose port `7860`.
    - CMD: `uvicorn src.main:app --host 0.0.0.0 --port 7860`.

## 3. Update Frontend
- [ ] Modify `apps/web/src/components/Chat/index.tsx`.
    - Mark the URL constant for update.

## 4. User Instructions
- [ ] Provide a checklist:
    1.  Push changes to GitHub.
    2.  Create new Space on Hugging Face -> Select "Docker".
    3.  Connect the GitHub repository.
    4.  Add Secrets in Settings -> Repository Secrets (`GEMINI_API_KEY`, `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`).
    5.  Copy the Space URL (Embed URL or Direct URL).
