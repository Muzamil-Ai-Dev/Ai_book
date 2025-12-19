# Tasks: AI-Native Textbook Visual & UX Redesign

**Input**: Design documents from `/specs/006-book-redesign/`  
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: Visual inspection and accessibility audits (Lighthouse) are the primary verification methods as per the specification. No automated TDD suite requested.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and routing configuration

- [X] T001 Update `docs.routeBasePath` to `'/modules'` in `apps/web/docusaurus.config.ts`
- [X] T002 [P] Configure `navbar` and `footer` base structure in `apps/web/docusaurus.config.ts`
- [X] T003 [P] Add Google Font imports (Inter, JetBrains Mono) to `stylesheets` in `apps/web/docusaurus.config.ts`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core design system variables that all UI components depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Implement color palette variables in `apps/web/src/css/custom.css`
- [X] T005 Implement typography and base font size variables in `apps/web/src/css/custom.css`
- [X] T006 [P] Implement base component styles (buttons, rounded corners) in `apps/web/src/css/custom.css`
- [X] T007 Define dark mode specific overrides in `apps/web/src/css/custom.css`

**Checkpoint**: Foundation ready - visual design system is established.

---

## Phase 3: User Story 1 - Professional First Impression (Priority: P1) 🎯 MVP

**Goal**: Create a distinct, polished landing page at the site root.

**Independent Test**: Navigate to `/` and verify the Hero section and marketing layout are present and visually distinct from the documentation.

### Implementation for User Story 1

- [X] T008 [P] [US1] Create the Landing Page component in `apps/web/src/pages/index.tsx`
- [X] T009 [P] [US1] Create the CSS module for landing page styling in `apps/web/src/pages/index.module.css`
- [X] T010 [US1] Implement the Hero Section (Title, Subtitle, CTA) in `apps/web/src/pages/index.tsx`
- [X] T011 [US1] Implement "What you will learn" and "Modules Overview" sections in `apps/web/src/pages/index.tsx`
- [X] T012 [US1] Design and implement a modern text/icon logo in `apps/web/static/img/logo.svg`
- [X] T013 [US1] Verify "Start Learning" CTA correctly routes to `/modules/01-intro/` (or current first module)

**Checkpoint**: At this point, the site has a professional "2025" entry point.

---

## Phase 4: User Story 2 - Premium Reading Experience (Priority: P1)

**Goal**: Optimize the technical content reading area for focus and comfort.

**Independent Test**: Open any module chapter and verify line length, typography, and sidebar hierarchy.

### Implementation for User Story 2

- [X] T014 [US2] Enforce max reading width (850px) on the documentation container in `apps/web/src/css/custom.css`
- [X] T015 [US2] Style the `DocSidebar` for clear visual separation and active-state indicators in `apps/web/src/css/custom.css`
- [X] T016 [P] [US2] Customize code block typography and padding in `apps/web/src/css/custom.css`
- [X] T017 [P] [US2] Adjust heading styles and vertical spacing in `apps/web/src/css/custom.css`
- [X] T018 [US2] Swizzle and customize the `DocItem` layout if necessary for structural refinements in `apps/web/src/components/`

**Checkpoint**: The core reading experience is distraction-free and premium.

---

## Phase 5: User Story 3 - Responsive Brand Consistency (Priority: P2)

**Goal**: Ensure the premium look persists across all device sizes.

**Independent Test**: Use browser inspector to verify mobile navigation and layout at 375px width.

### Implementation for User Story 3

- [X] T019 [US3] Implement mobile-specific navigation overrides in `apps/web/src/css/custom.css`
- [X] T020 [US3] Ensure responsive padding and font scaling for the landing page in `apps/web/src/pages/index.module.css`
- [X] T021 [P] [US3] Optimize image and SVG logo scaling for small screens in `apps/web/src/css/custom.css`

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final refinements and quality checks

- [X] T022 [P] Audit all colors for WCAG AA accessibility contrast ratios
- [X] T023 Run Google Lighthouse performance and accessibility audit
- [X] T024 [P] Cleanup unused default Docusaurus CSS and assets
- [X] T025 Final visual regression check against all success criteria in `spec.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Must complete T001 first to enable the landing page.
- **Foundational (Phase 2)**: Must complete to ensure US1 and US2 have the correct colors/fonts.
- **User Stories (Phase 3+)**: 
  - US1 (Landing Page) and US2 (Reading Experience) can be worked on in parallel.
  - US3 (Responsive) depends on US1 and US2 being mostly complete.

### Parallel Opportunities

- T002 and T003 (Navbar/Footer vs Fonts)
- T006 and T007 (Base styles vs Dark mode)
- US1 (Phase 3) and US2 (Phase 4) can proceed in parallel once Phase 2 is done.

---

## Implementation Strategy

### MVP First (Landing Page + Basic Theming)

1. Complete Phase 1 & 2 (Setup & Foundation).
2. Complete Phase 3 (Landing Page).
3. **STOP and VALIDATE**: Verify the new "2025 AI Startup" look is active on the home page.

### Incremental Delivery

1. Foundation ready.
2. Homepage ready (MVP!).
3. Reading experience optimized.
4. Mobile responsiveness finalized.

---

## Notes

- All paths are relative to the repository root.
- CSS overrides primarily target Infima variables to ensure maximum compatibility with Docusaurus updates.
