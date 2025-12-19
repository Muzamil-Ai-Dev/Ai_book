# Implementation Plan: AI-Native Textbook Visual & UX Redesign

**Feature**: `006-book-redesign`
**Created**: 2025-12-19
**Status**: Draft

## Technical Context

The project is built with **Docusaurus 3.x**. The current documentation is served at the root `/`. To implement a modern landing page, the docs must be moved to a sub-route (`/modules`), allowing for a custom React-based homepage.

### Constraints & Invariants
- **No Content Changes**: Content in `modules/` must remain untouched.
- **Docusaurus Native**: Prefer Docusaurus/Infima patterns over heavy external frameworks.
- **Routing**: Must not break existing deep links (redirects may be needed if `/` links were shared, but for this redesign, the new landing page will occupy `/`).

## Constitution Check

| Principle | Adherence |
|-----------|-----------|
| I. Spec-Driven | Redesign is driven by the 006-book-redesign spec. |
| III. Atomic Specs | This spec focuses exclusively on Visual & UX. |
| VII. Canonical Formats | Maintains MDX for content and standard Docusaurus theme patterns. |

## Implementation Phases

### Phase 1: Structural Setup & Routing
- **Objective**: Prepare the site for a custom homepage.
- **Tasks**:
  - Update `docusaurus.config.ts`: Set `docs.routeBasePath` to `/modules`.
  - Update `sidebars.ts`: Ensure the `moduleSidebar` is correctly structured.
  - Verification: `npm run build` and check that `/modules/01-intro/` is reachable.

### Phase 2: Design System (The "Look")
- **Objective**: Establish the 2025 "Futuristic Academic" aesthetic.
- **Tasks**:
  - Implement CSS variables in `src/css/custom.css` (Colors, Typography, Spacing).
  - Add Google Font imports in `docusaurus.config.ts`.
  - Style common elements (Buttons, Code blocks, Navbar).
- **Artifacts**: Updated `custom.css`.

### Phase 3: Custom Landing Page
- **Objective**: Create the marketing entry point.
- **Tasks**:
  - Implement `src/pages/index.tsx` with a Hero section and module highlights.
  - Design a simple logo placeholder or text-brand.
- **Artifacts**: `src/pages/index.tsx`, `index.module.css`.

### Phase 4: Reading Experience & UX
- **Objective**: Refine the long-form technical reading area.
- **Tasks**:
  - Set max-width on the documentation container.
  - Refine sidebar styling (collapsible indicators, active states).
  - Swizzle components if necessary for sticky progress or layout tweaks.

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Routing Conflicts | Medium | Use `routeBasePath: '/modules'` and verify all links. |
| Performance Drop | Low | Avoid heavy animations; use standard Docusaurus optimizations. |
| Visual Regression | Low | Visual inspection against the original spec goals. |