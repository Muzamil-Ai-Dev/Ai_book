# Plan: Render Deployment

## 1. Prepare Backend for Production
- [ ] Update `apps/api/src/main.py` to include `https://Muzamil-Ai-Dev.github.io` in CORS `allow_origins`.
- [ ] Verify `apps/api/requirements.txt` is complete (already checked, looks standard but good to re-verify if needed, though we will trust the existing one for now).

## 2. Infrastructure as Code
- [ ] Create `render.yaml` in the root directory.
    - Define a `web` service.
    - Root directory: `.` (or handle paths in commands).
    - Build command: `pip install -r apps/api/requirements.txt`
    - Start command: `uvicorn src.main:app --app-dir apps/api --host 0.0.0.0 --port $PORT` (Adjusted to run from root context if needed, or set `root` in render.yaml).

## 3. Update Frontend
- [ ] Modify `apps/web/src/components/Chat/index.tsx`.
    - Replace the placeholder `https://your-backend-api-url.com` with the *anticipated* Render URL.
    - Note: Since we don't have the URL yet, we might need to ask the user to provide it after deployment, OR we can set a pattern like `https://ai-book-api.onrender.com` if we can claim that name.
    - *Better approach:* Make it easy to change. For now, we will update the code to clearly indicate where the URL goes, or use a Docusaurus `customField`.
    - Let's use a hardcoded value that the user is instructed to update, as environment variables in static sites are baked in at build time.

## 4. User Instructions
- [ ] Provide a checklist for the user to:
    1.  Push changes to GitHub.
    2.  Go to Render.com -> Blueprints -> New Blueprint Instance.
    3.  Connect the repo.
    4.  Set the environment variables in the Render dashboard.
    5.  Get the resulting URL.
    6.  Update the frontend `API_URL` constant if the auto-generated name differs.
