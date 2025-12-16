---
id: 004
title: Update Constitution v1.1.0
stage: constitution
date: 2025-12-16
surface: agent
model: gemini
feature: none
branch: master
user: user
command: /sp.constitution ... (update)
labels:
  - governance
  - architecture
  - security
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: []
---

# Prompt

Update to v1.1.0. Add Principle VIII: Bounded Knowledge (RAG grounded only in book). Add Principle
IX: Zero Secrets (strict .env usage). Define Directory Standard: apps/web, apps/api,
content/modules.

# Response

Updated `.specify/memory/constitution.md` to Version 1.1.0. Added "Bounded Knowledge" to restrict AI
scope. Added "Zero Secrets" for security. Defined Monorepo structure: `apps/web`, `apps/api`,
`content/modules`.
