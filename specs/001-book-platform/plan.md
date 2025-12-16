# Implementation Plan: Book Platform Infrastructure

**Branch**: `001-book-platform` | **Date**: 2025-12-16 | **Spec**:
[specs/001-book-platform/spec.md](./spec.md) **Input**: Feature specification from
`specs/001-book-platform/spec.md`

## Summary

Set up the core Docusaurus infrastructure in a monorepo structure (`apps/web`, `content/modules`).
Configure the sidebar for the 8 mandatory modules, implement GitHub Pages deployment, and ensure
content is read from the external directory.

## Technical Context

**Language/Version**: Node.js 18+ (LTS), React 18 **Primary Dependencies**: Docusaurus v3, Prettier
**Storage**: File-based (Markdown/MDX) **Testing**: Build verification, Link checking **Target
Platform**: GitHub Pages (Static Web) **Project Type**: Monorepo (npm workspaces) **Performance
Goals**: Build < 3 mins, Perfect Lighthouse Accessibility score **Constraints**: Strict separation
of content and code

## Content Seeding Strategy

To ensure "Content as First-Class Artifact" (Constitution Principle II) is met immediately:

- A script or task will generate the 8 mandatory Markdown files in `content/modules`.
- Each file will be seeded with valid Frontmatter (`id`, `title`, `sidebar_position`) and a "Under
  Construction" body.
- This ensures the `sidebars.js` autogeneration works immediately upon first build.

## Constitution Check

_GATE: Must pass before Phase 0 research. Re-check after Phase 1 design._

- [x] **Spec-Driven**: Created via `/sp.specify` and now `/sp.plan`.
- [x] **Content First**: Enforced via `content/modules` separation.
- [x] **Atomic**: Focuses ONLY on platform, no RAG/Auth yet.
- [x] **Verifiable**: Success defined by build/deploy capability.
- [x] **Monorepo Standard**: Adheres to `apps/web` structure.

## Project Structure

### Documentation (this feature)

```text
specs/001-book-platform/
├── plan.md              # This file
├── research.md          # Tech decisions
├── data-model.md        # Content schema
├── quickstart.md        # N/A for this infra feature
├── contracts/           # N/A for static site
└── tasks.md             # To be generated
```

### Source Code (repository root)

```text
Ai_book/
├── package.json          # Workspace root
├── apps/
│   └── web/              # Docusaurus Project
│       ├── docusaurus.config.js
│       ├── src/
│       ├── sidebars.js
│       └── static/
├── content/
│   └── modules/          # Markdown Source
│       ├── 01-intro.md
│       ├── 02-ros2.md
│       └── ...
└── .github/
    └── workflows/        # Deploy scripts
```

**Structure Decision**: Standard Monorepo with explicit `apps/web` vs `content` separation.

## Complexity Tracking

_None. The structure adheres strictly to the Constitution._
