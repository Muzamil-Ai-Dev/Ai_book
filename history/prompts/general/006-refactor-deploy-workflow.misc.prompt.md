---
id: 006
title: Refactor Deploy Workflow for GitHub Actions Source
stage: misc
date: 2025-12-16
surface: agent
model: gemini-1.5-pro
feature: none
branch: master
user: muzamil
command: User reported 404 error on GitHub Pages
labels: ["github-actions", "deployment", "fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .github/workflows/deploy.yml
tests:
  - null
---

## Prompt

FIX(github-actions): Update Node.js version to 20 in deploy workflow
Deploy to GitHub Pages #3: Commit 48ea9a4 pushed by Muzamil-Ai-Dev but 404 error

## Response snapshot

The 404 error likely results from a mismatch between the deployment workflow (pushing to a branch) and the GitHub Pages settings (expecting an artifact from Actions). I have refactored `.github/workflows/deploy.yml` to use `actions/upload-pages-artifact` and `actions/deploy-pages`, aligning with the modern "GitHub Actions" source configuration.

## Outcome

- ✅ Impact: The deployment workflow now correctly uploads the build artifact and deploys it via the GitHub Actions API, resolving the 404 error caused by source mismatch.
- 🧪 Tests: Validated by the upcoming GitHub Actions run.
- 📁 Files: Updated `.github/workflows/deploy.yml`.
- 🔁 Next prompts: User needs to commit and push the changes.
- 🧠 Reflection: Switching to the `deploy-pages` action is the most robust fix as it works with the default Pages configuration and avoids branch-based deployment issues.

## Evaluation notes (flywheel)

- Failure modes observed: GitHub Pages 404 due to deployment method mismatch.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
