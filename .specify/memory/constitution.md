<!--
Sync Impact Report:
- Version change: 0.0.0 -> 1.0.0
- Modified principles: Established 7 core principles based on Spec-Driven Development for AI Textbooks.
- Added sections: Architecture & Technology Stack, Content Requirements.
- Templates requiring updates: N/A (Initial Setup).
-->
# AI-Native Textbook for Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Spec-Driven Everything
Everything is spec-driven — infrastructure, writing, AI agents, and deployment. No ad-hoc development or manual configuration is permitted without an accompanying spec.

### II. Content as First-Class Artifact
Content creation (chapters) is treated as a first-class artifact. It must be explicitly specified, written (not placeholders), and verified like code.

### III. Atomic & Composable Specs
Specs are small, composable, and independently executable. No spec may mix concerns (e.g., infrastructure ≠ content ≠ AI ≠ auth). Each spec must do one thing well.

### IV. Verifiable Outputs
Every spec must produce verifiable outputs. This includes compiled sites, passing tests, reachable APIs, or rendered chapters. If it cannot be verified, it is not done.

### V. Incremental Deployability
The system must be incrementally deployable at each major milestone. A working version of the book (even if partial) should be deployable after the platform spec, content spec, etc.

### VI. Pedagogical Clarity
Prefer clarity and pedagogy over marketing language. Assume students know AI basics but are beginners in robotics. The goal is education, not hype.

### VII. Canonical Formats
Docusaurus Markdown is the canonical content format. All content generation and management must adhere to this standard.

## Architecture & Technology Stack

**Target Architecture:**
The project is executed via sequential, isolated specs:
1. **Book Platform:** Docusaurus setup.
2. **Book Content:** Chapter writing and assets.
3. **AI Retrieval:** RAG Chatbot (FastAPI, Qdrant, OpenAI).
4. **Robotics Simulation:** Context and simulation integration.
5. **User Auth:** Authentication & Personalization (optional/bonus).
6. **Translation:** Urdu translation (optional/bonus).
7. **Deployment:** GitHub Pages / Vercel.

**Required Stack:**
- **Frontend/Static Site:** Docusaurus (React/Markdown).
- **AI/Backend:** FastAPI, OpenAI Agents/ChatKit, Qdrant Cloud (Free Tier), Neon Serverless Postgres.
- **Bonus:** better-auth for authentication.

## Content Requirements

**Mandatory Modules:**
- Introduction to Physical AI & Embodied Intelligence
- ROS 2 Fundamentals
- Gazebo & Unity Simulation
- NVIDIA Isaac Platform
- Vision-Language-Action (VLA)
- Humanoid Robotics & Locomotion
- Conversational Robotics
- Capstone Project Guidance

**Chapter Structure:**
Each chapter MUST include:
- Learning objectives
- Concept explanations
- Diagrams or visual descriptions
- Hands-on or thought exercises
- Preparation for later modules

## Governance

The constitution governs all future specs, plans, and tasks. It supersedes individual spec preferences.
- **Amendments:** Must be documented via a version bump and rationale.
- **Compliance:** All generated specs must reference these principles.
- **Validation:** `/sp.specify` calls must align with the architecture defined here.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16