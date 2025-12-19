---
id: 005
title: Generate Tasks Physical AI Content
stage: tasks
date: 2025-12-16
surface: agent
model: gemini
feature: book-content
branch: 002-book-content
user: muzamil
command: /sp.tasks
labels: ["tasks", "workflow", "content"]
links:
  spec: specs/002-book-content/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-book-content/tasks.md
tests: []
---

## Prompt

$ARGUMENTS

## Response snapshot

Generated tasks.md with 6 phases, covering setup, foundation, content creation (Concepts, Hands-on, Capstone), and polish.

## Outcome

- ✅ Impact: Clear, actionable roadmap for content creation.
- 🧪 Tests: N/A
- 📁 Files: tasks.md
- 🔁 Next prompts: /sp.implement
- 🧠 Reflection: The task breakdown separates "concepts" from "hands-on", which allows for an iterative writing process (drafting theory first, then adding labs).

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): none
