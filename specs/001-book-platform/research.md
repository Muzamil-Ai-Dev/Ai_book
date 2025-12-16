# Research & Decisions: Book Platform Infrastructure

**Feature**: Book Platform Infrastructure **Date**: 2025-12-16

## Decisions

### 1. Docusaurus Version

- **Decision**: Use Docusaurus v3 (latest stable).
- **Rationale**: Provides latest React support, MDX v3, and better build performance. Essential for
  long-term maintenance.
- **Alternatives Considered**: v2 (legacy, no reason to use), v4 (alpha, too unstable).

### 2. Monorepo Tooling

- **Decision**: Use `npm workspaces` (or simple directory separation if dependencies are minimal).
- **Rationale**: The Constitution mandates `apps/web` and `apps/api`. A simple `package.json` at
  root defining workspaces is the standard way to manage this in Node.js ecosystems without
  over-engineering (like Nx or Turborepo for this scale).
- **Alternatives Considered**: Lerna, Nx, Turborepo (overkill for this stage).

### 3. Content Plugin Configuration

- **Decision**: Configure `@docusaurus/plugin-content-docs` with `path: '../../content/modules'`.
- **Rationale**: Directly addresses Constitution Principle II and Spec FR-007 to keep content
  strictly separated from the presentation layer.
- **Alternatives Considered**: Symlinks (fragile on Windows), Copy scripts (prone to sync errors).

### 4. Deployment Strategy

- **Decision**: GitHub Actions for CI/CD + `gh-pages` branch.
- **Rationale**: Automated, verifiable, and free. Fits the "Incremental Deployability" principle.
- **Alternatives Considered**: Vercel (Constitution mentions it, but GH Pages is the primary target
  for the "textbook" artifact; Vercel is good for the dynamic parts later). We will start with GH
  Pages as per User Story 2.

### 5. Linting Stack

- **Decision**: Prettier with `prettier-plugin-sh` and standard markdown support.
- **Rationale**: Low config, high impact.
- **Alternatives Considered**: ESLint for Markdown (too noisy).

## Unresolved Questions (Resolved)

- _None. The spec was very specific._
