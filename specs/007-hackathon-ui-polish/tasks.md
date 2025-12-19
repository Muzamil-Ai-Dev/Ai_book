# Tasks: Hackathon-Winning UI & UX Polish

**Input**: Design documents from `/specs/007-hackathon-ui-polish/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, quickstart.md

**Tests**: N/A (Visual/Performance verification as per spec.md)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `apps/web/src/`
- Paths shown below assume root as `apps/web/` unless specified

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Install `framer-motion` dependency in `apps/web/package.json`
- [X] T002 Define neon/glass CSS variables and design tokens in `apps/web/src/css/custom.css`
- [X] T003 [P] Ensure `docusaurus.config.ts` correctly references `custom.css`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [X] T004 Create foundational animation classes (entrance/hover) in `apps/web/src/css/animations.css`
- [X] T005 [P] Setup reusable Framer Motion variants for common UI transitions

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - The "Wow" Factor Landing Page (Priority: P1) 🎯 MVP

**Goal**: Deliver immediate visual impact with glassmorphism, background glow blobs, and entrance animations.

**Independent Test**: Verify glassmorphism on Navbar/Cards, animating background blobs in Hero, and smooth scroll entrance animations on the home page.

### Implementation for User Story 1

- [X] T006 [US1] Apply Glassmorphism CSS (`backdrop-filter`) to Navbar in `apps/web/src/css/custom.css`
- [X] T007 [P] [US1] Create Glassmorphic Card component wrapper in `apps/web/src/components/GlassCard/index.tsx`
- [X] T008 [P] [US1] Implement animating background glow blobs component in `apps/web/src/components/GlowBlobs/index.tsx`
- [X] T009 [US1] Integrate `GlowBlobs` into the Hero section background in `apps/web/src/pages/index.tsx`
- [X] T010 [US1] Wrap Landing Page sections with entrance animation containers in `apps/web/src/pages/index.tsx`

**Checkpoint**: Landing Page should now have the "Wow" factor and be testable independently.

---

## Phase 4: User Story 2 - Premium Interactive Reading (Priority: P1)

**Goal**: Enhance the reading experience with bespoke scrollbars and high-fidelity code blocks.

**Independent Test**: Verify custom slim scrollbar and macOS-style code block headers with language labels and integrated copy buttons.

### Implementation for User Story 2

- [X] T011 [P] [US2] Implement custom slim, neon scrollbar styling in `apps/web/src/css/custom.css`
- [X] T012 [US2] Swizzle the Docusaurus `CodeBlock` component to `apps/web/src/theme/CodeBlock/`
- [X] T013 [US2] Implement macOS-style "traffic light" buttons and language tag in `apps/web/src/theme/CodeBlock/index.tsx`
- [X] T014 [US2] Style the custom CodeBlock header (glassy effect) in `apps/web/src/theme/CodeBlock/styles.module.css`

**Checkpoint**: Reading interface should feel specialized and "bespoke".

---

## Phase 5: User Story 3 - Dark-Mode "Glow" Branding (Priority: P2)

**Goal**: Create a cohesive brand identity using neon cyan glow accents on interactive elements.

**Independent Test**: Verify glow effects on primary buttons and active sidebar links.

### Implementation for User Story 3

- [X] T015 [US3] Apply neon cyan glow box-shadows to primary buttons in `apps/web/src/css/custom.css`
- [X] T016 [US3] Implement glowing active indicator for the module sidebar in `apps/web/src/css/custom.css`

**Checkpoint**: All brand accents should consistently use the neon-glow theme.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Fine-tuning typography, performance, and accessibility.

- [X] T017 Refine typography tracking (letter-spacing) and leading (line-height) in `apps/web/src/css/custom.css`
- [X] T018 Perform animation performance audit (60fps check) as per SC-002
- [X] T019 Conduct accessibility contrast audit for glassmorphic backgrounds (WCAG AA) as per SC-004
- [X] T020 Run final verification using `specs/007-hackathon-ui-polish/quickstart.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Must complete first to provide tokens/dependencies.
- **Foundational (Phase 2)**: Depends on Phase 1 - Blocks all User Stories.
- **User Stories (Phase 3-5)**: Can proceed in parallel after Phase 2 is complete.
- **Polish (Phase 6)**: Final step after implementation.

### User Story Dependencies

- **US1, US2, US3**: All are independent of each other but depend on the same foundational CSS/Dependencies.

### Parallel Opportunities

- T003 can run with T001/T002.
- T005 can run with T004.
- T007 and T008 can be developed in parallel.
- Once Foundations are in place, US1 (Landing Page) and US2 (Reading Interface) can be implemented by different agents/developers.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1 & 2.
2. Implement User Story 1 (Landing Page).
3. **STOP and VALIDATE**: Ensure the landing page achieves the "Wow" factor.

### Incremental Delivery

1. Foundation ready.
2. US1 (Landing Page) -> Deploy/Demo.
3. US2 (Interactive Reading) -> Deploy/Demo.
4. US3 (Branding) -> Final Polish.