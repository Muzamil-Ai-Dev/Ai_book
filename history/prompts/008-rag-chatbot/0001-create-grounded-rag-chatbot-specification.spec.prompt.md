---
id: "0001"
title: "Create Grounded RAG Chatbot Specification"
stage: "spec"
date: "2025-12-19"
surface: "agent"
model: "gemini-2.0-flash-thinking-exp"
feature: "008-rag-chatbot"
branch: "008-rag-chatbot"
user: "Muzamil"
command: "/sp.specify"
labels: ["rag", "chatbot", "specification", "grounding"]
links:
  spec: "specs/008-rag-chatbot/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - specs/008-rag-chatbot/spec.md
  - specs/008-rag-chatbot/checklists/requirements.md
tests:
  - "Specification Quality Checklist"
---

## Prompt

Create a specification for implementing a Retrieval-Augmented Generation (RAG) chatbot for the AI-Native Textbook for Physical AI & Humanoid Robotics. This chatbot is a core AI-Native feature and must demonstrate grounded, non-hallucinating intelligence. ──────────────────────────────── PURPOSE ──────────────────────────────── Enable users to ask questions and receive answers strictly grounded in the textbook’s Markdown content. The chatbot must: • Answer ONLY from book content • Refuse to answer when information is missing • Expose retrieved sources for transparency • Demonstrate AI-Native learning behavior ──────────────────────────────── ARCHITECTURE REQUIREMENTS ──────────────────────────────── • Backend: FastAPI • Orchestration: OpenAI ChatKit SDK • LLM for response generation: Gemini (via Gemini API key) • Retrieval: Embeddings + vector database • Grounding: Context-7 MCP server (mandatory) • Content source: Docusaurus Markdown ──────────────────────────────── CONTENT SOURCE ──────────────────────────────── • Canonical source: content/modules/ • Each Markdown chapter is a retrievable unit • Content must be chunked with metadata: - module - chapter - heading - file path ──────────────────────────────── CORE COMPONENTS ──────────────────────────────── 1. INGESTION PIPELINE • Read Markdown files from content/modules/ • Chunk content deterministically • Generate embeddings • Store embeddings in vector DB • Preserve source metadata Endpoint: POST /ingest ──────────────────────────────── 2. RETRIEVAL + GROUNDING • Retrieve top-K relevant chunks • Pass retrieved context through Context-7 MCP • Apply relevance filtering • If confidence is low → refuse Context-7 must ensure: • No external knowledge • No speculative answers • Hard grounding to retrieved text ──────────────────────────────── 3. RESPONSE GENERATION • OpenAI ChatKit is used to orchestrate the flow: - user question - retrieved context - refusal rules • Gemini LLM is used ONLY for final response generation • Gemini must not receive any information outside retrieved context ──────────────────────────────── 4. QUERY ENDPOINT Endpoint: POST /query Behavior: • Accepts user question • Retrieves relevant content • Applies Context-7 grounding • Uses ChatKit + Gemini to generate response • Returns: - answer - sources used - refusal reason (if applicable) ──────────────────────────────── NON-FUNCTIONAL REQUIREMENTS ──────────────────────────────── • Deterministic chunking • No hardcoded secrets • Environment variables only • Clear logging for: - retrieved chunks - grounding decisions • Simple JSON responses (no UI logic) ──────────────────────────────── VERIFICATION CRITERIA ──────────────────────────────── • Questions answerable from the book → correct grounded response • Questions not in the book → refusal • Logs show retrieved source files • Gemini responses never exceed provided context ──────────────────────────────── OUTCOME ──────────────────────────────── A production-ready, AI-Native RAG chatbot that proves the textbook is interactive, grounded, and non-hallucinating. This system is evaluated as a core hackathon deliverable.

## Response snapshot

Created a comprehensive feature specification for the Grounded RAG Chatbot. The specification includes four user scenarios (covering core answering, refusal, transparency, and conversational learning), a dedicated section for Architectural Constraints (FastAPI, Gemini, Context-7), and detailed functional requirements. It explicitly defines the mandatory metadata for chunking (module, chapter, heading, file path) and introduces a cross-verification grounding check. A quality checklist was used to validate the spec before and after these refinements.

## Outcome

- ✅ Impact: High - Defines the core AI feature and its architectural boundaries for the hackathon.
- 🧪 Tests: Validated against Specification Quality Checklist.
- 📁 Files: Created spec.md and requirements checklist.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Successfully balanced the need for technology-agnostic requirements with the user's explicit architectural constraints. The addition of conversational memory better captures the "AI-Native learning" goal.

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec contained implementation details; corrected in iteration.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null