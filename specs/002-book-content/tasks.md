# Tasks: Physical AI & Humanoid Robotics Textbook Content

**Branch**: `002-book-content` | **Phase**: Implementation | **Status**: Pending

## Phase 1: Setup
**Goal**: Initialize directory structure and validation tooling.

- [x] T001 Create root content directory structure `content/modules`
- [x] T002 Create subdirectories for all 8 modules (01-intro to 08-capstone) in `content/modules/`
- [x] T003 Implement `scripts/validate-content.js` to enforce FR-002 (5 required sections) and FR-006 (local assets)
- [x] T004 Add `validate-content` script to `package.json` in root or `apps/web`
- [x] T005 Create asset directory structure `apps/web/static/img/modules/` with subfolders for each module

## Phase 2: Foundational
**Goal**: Establish Docusaurus navigation and sidebar structure.

- [x] T006 [P] Create `content/modules/01-intro/_category_.json` with metadata
- [x] T007 [P] Create `content/modules/02-ros2/_category_.json` with metadata
- [x] T008 [P] Create `content/modules/03-gazebo-unity/_category_.json` with metadata
- [x] T009 [P] Create `content/modules/04-isaac/_category_.json` with metadata
- [x] T010 [P] Create `content/modules/05-vla/_category_.json` with metadata
- [x] T011 [P] Create `content/modules/06-humanoid/_category_.json` with metadata
- [x] T012 [P] Create `content/modules/07-conversational/_category_.json` with metadata
- [x] T013 [P] Create `content/modules/08-capstone/_category_.json` with metadata
- [ ] T014 Configure `apps/web/docusaurus.config.ts` to source content from `content/modules`.
- [x] T015 Implement `scripts/sync-content.js` to copy/symlink `content/modules` to `apps/web/docs`.
- [x] T016 Add `sync-content` script as a pre-build/pre-start hook in `apps/web/package.json`.

## Phase 3: Learner Studying Concepts (User Story 1)
**Goal**: Create core conceptual content for all modules with pedagogical clarity.

- [x] T017 [P] [US1] Write `01-intro/index.md` (Module Overview)
- [x] T018 [P] [US1] Write `01-intro/01-embodied-ai.md` (Concepts & Objectives)
- [x] T019 [P] [US1] Write `02-ros2/index.md` (Module Overview)
- [x] T020 [P] [US1] Write `02-ros2/01-nodes-topics.md` (Concepts & Objectives)
- [x] T021 [P] [US1] Write `03-gazebo-unity/index.md` (Module Overview)
- [x] T022 [P] [US1] Write `03-gazebo-unity/01-simulation-basics.md` (Concepts & Objectives)
- [x] T023 [P] [US1] Write `04-isaac/index.md` (Module Overview)
- [x] T024 [P] [US1] Write `04-isaac/01-usd-omniverse.md` (Concepts & Objectives) w/ Hardware Warnings
- [x] T025 [P] [US1] Write `05-vla/index.md` (Module Overview)
- [x] T026 [P] [US1] Write `05-vla/01-transformers-robotics.md` (Concepts & Objectives)
- [x] T027 [P] [US1] Write `06-humanoid/index.md` (Module Overview)
- [x] T028 [P] [US1] Write `06-humanoid/01-bipedal-locomotion.md` (Concepts & Objectives)
- [x] T029 [P] [US1] Write `07-conversational/index.md` (Module Overview)
- [x] T030 [P] [US1] Write `07-conversational/01-llm-integration.md` (Concepts & Objectives)
- [x] T031 [P] [US1] Write `08-capstone/index.md` (Module Overview)

## Phase 4: Hands-on Labs (User Story 2)
**Goal**: Add practical exercises and code snippets to existing chapters.

- [x] T022 [US2] Add Hands-on Exercises to `02-ros2/01-nodes-topics.md` (Python/C++ examples)
- [x] T023 [US2] Add Hands-on Exercises to `03-gazebo-unity/01-simulation-basics.md` (Unity Setup)
- [x] T024 [US2] Add Hands-on Exercises to `04-isaac/01-usd-omniverse.md` (Isaac Sim Setup)
- [x] T025 [US2] Add Hands-on Exercises to `05-vla/01-transformers-robotics.md` (Model Inference)
- [x] T026 [US2] Add Hands-on Exercises to `06-humanoid/01-bipedal-locomotion.md` (Gait Control)

## Phase 5: Capstone Prep (User Story 3)
**Goal**: Connect modules to the final project and write the Capstone guide.

- [x] T027 [US3] Add Capstone Prep section to `01-intro/01-embodied-ai.md`
- [x] T028 [US3] Add Capstone Prep section to `02-ros2/01-nodes-topics.md`
- [x] T029 [US3] Add Capstone Prep section to `03-gazebo-unity/01-simulation-basics.md`
- [x] T030 [US3] Write full content for `08-capstone/01-project-spec.md`

## Phase 6: Polish
**Goal**: Final validation and asset check.

- [x] T031 Run `npm run validate-content` to ensure 100% compliance
- [x] T032 Verify Docusaurus build `npm run build`
- [x] T033 Check for broken links using Docusaurus build output

## Dependencies

- Phase 2 depends on Phase 1 (Directory structure)
- Phase 3 depends on Phase 2 (Navigation structure)
- Phase 4 depends on Phase 3 (Base content existence)
- Phase 5 depends on Phase 3 (Base content existence)

## Implementation Strategy

- **MVP**: Complete Phase 1, 2, and T015-T016 (First two modules).
- **Parallelism**: Writers can work on different modules in Phase 3/4/5 simultaneously once Phase 2 is done.
