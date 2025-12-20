# Tasks: Finalize Hugging Face Deployment

## 1. Configure GitHub Action
- [x] Create `.github/workflows/sync-to-hf.yml`.
- [ ] **Edit the file**: Open `.github/workflows/sync-to-hf.yml` and replace `YOUR_HF_USERNAME/YOUR_SPACE_NAME` with your actual Space path (e.g., `muzamil/ai-book-api`).

## 2. Get Hugging Face Token
- [ ] Go to [Hugging Face Settings -> Tokens](https://huggingface.co/settings/tokens).
- [ ] Create a new token (Type: **Write**).
- [ ] Copy the token (starts with `hf_...`).

## 3. Add Secret to GitHub
- [ ] Go to your GitHub Repo -> **Settings** -> **Secrets and variables** -> **Actions**.
- [ ] Click **New repository secret**.
- [ ] Name: `HF_TOKEN`.
- [ ] Value: Paste your Hugging Face token.
- [ ] Click **Add secret**.

## 4. Deploy
- [ ] Push the changes to GitHub. The action will run automatically.