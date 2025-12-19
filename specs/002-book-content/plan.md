# Implementation Plan: Physical AI & Humanoid Robotics Textbook Content

**Branch**: `002-book-content` | **Date**: 2025-12-16 | **Spec**: [specs/002-book-content/spec.md](../spec.md)
**Input**: Feature specification from `specs/002-book-content/spec.md`

## Summary

This plan outlines the creation of the core educational content for the "Physical AI & Humanoid Robotics" textbook. The primary goal is to produce high-quality, pedagogical Markdown/MDX modules covering topics from embodied intelligence to humanoid locomotion. The content will be structured for Docusaurus, ensuring modularity, renderability, and adherence to the "Content as First-Class Artifact" principle.

## Technical Context

**Language/Version**: Markdown / MDX (Docusaurus compatible)
**Primary Dependencies**: Docusaurus 3.x (already set up in `001-book-platform`)
**Storage**: Git-based file storage (local repository)
**Testing**: 
  - Docusaurus build verification (`npm run build` with `onBrokenLinks: 'throw'`)
  - Custom Content Validator (`scripts/validate-content.js`) for frontmatter & asset integrity
**Target Platform**: Web (Static Site Generation)
**Project Type**: Documentation / Educational Content
**Performance Goals**: Fast build times for static content; optimized images
**Constraints**: No external asset hotlinking; hardware requirements for specific modules (NVIDIA Isaac)
**Scale/Scope**: 8 Core Modules, ~30-50 sub-chapters/pages

## Constitution Check

_GATE: Must pass before Phase 0 research. Re-check after Phase 1 design._

- [x] **Spec-Driven Everything**: Content is defined by `002-book-content/spec.md`.
- [x] **Content as First-Class Artifact**: Plan explicitly treats chapters as artifacts to be built and verified.
- [x] **Atomic & Composable Specs**: Focuses solely on content creation, separating it from platform infrastructure (001) and AI agents (003).
- [x] **Verifiable Outputs**: Success is defined by a successful Docusaurus build and render.
- [x] **Incremental Deployability**: The partial book can be deployed immediately after content merge.
- [x] **Pedagogical Clarity**: Spec mandates learning objectives and concept explanations.
- [x] **Canonical Formats**: Strictly enforces Docusaurus Markdown.
- [x] **Bounded Knowledge**: Sets the foundation for the RAG system by creating the ground truth content.
- [x] **Zero Secrets in Code**: No code with secrets will be written; `.env` usage applies to platform, not static content content.

## Project Structure

### Documentation (this feature)

```text
specs/002-book-content/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output (Content Hierarchy)
├── quickstart.md        # Phase 1 output (Writing Guide)
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
content/
└── modules/
    ├── 01-intro/
    ├── 02-ros2/
    ├── 03-gazebo-unity/
    ├── 04-isaac/
    ├── 05-vla/
    ├── 06-humanoid/
    ├── 07-conversational/
    └── 08-capstone/

apps/web/
├── docs/               # Symlinked or copied from content/modules for Docusaurus
└── static/
    └── img/
        └── modules/    # Local asset storage

scripts/
└── validate-content.js # Automates SC-002 (Structure) and FR-006 (Assets) checks
```

**Structure Decision**: We will use a `content/modules` directory at the root to maintain separation between raw content and the Docusaurus app (`apps/web`). This aligns with the "Directory Standard" in the Constitution.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
| --------- | ---------- | ------------------------------------ |
| None      | N/A        | N/A                                  |