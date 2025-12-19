# Tasks: Expand Textbook Content

**Branch**: `003-expand-content` | **Phase**: Implementation | **Status**: Pending

## Phase 1: Tooling
**Goal**: Enforce "No Images" policy.

- [x] T001 Update `scripts/validate-content.js` to error on ANY image tag (`![]`, `<img`).
- [x] T002 Update `scripts/validate-content.js` to accept "Conceptual Visualization" as an alternative to "Visual Descriptions".

## Phase 2: Writing Batch 1 (Foundation)
**Goal**: ~11,000 words total.

- [x] T003 [US1] Rewrite `content/modules/01-intro/index.md` (Intro to Module)
- [x] T004 [US1] Rewrite `content/modules/01-intro/01-embodied-ai.md` (Deep dive: Embodied AI - 3000 words)
- [x] T005 [US2] Rewrite `content/modules/02-ros2/index.md` (Intro to Module)
- [in_progress] T006 [US2] Rewrite `content/modules/02-ros2/01-nodes-topics.md` (Deep dive: ROS 2 Architecture - 4000 words)

## Phase 3: Writing Batch 2 (Simulation)
**Goal**: ~10,000 words total.

- [x] T007 [US3] Rewrite `content/modules/03-gazebo-unity/index.md`
- [x] T008 [US3] Rewrite `content/modules/03-gazebo-unity/01-simulation-basics.md` (Deep dive: URDF/SDF & Physics Engines)
- [x] T009 [US3] Rewrite `content/modules/04-isaac/index.md`
- [x] T010 [US3] Rewrite `content/modules/04-isaac/01-usd-omniverse.md` (Deep dive: USD & Isaac Sim API)

## Phase 4: Writing Batch 3 (Intelligence & Body)
**Goal**: ~16,000 words total.

- [x] T011 [US3] Rewrite `content/modules/05-vla/index.md`
- [x] T012 [US3] Rewrite `content/modules/05-vla/01-transformers-robotics.md` (Deep dive: RT-1, RT-2, Transfomers)
- [x] T013 [US3] Rewrite `content/modules/06-humanoid/index.md`
- [x] T014 [US3] Rewrite `content/modules/06-humanoid/01-bipedal-locomotion.md` (Deep dive: ZMP, LIPM, MPC)

## Phase 5: Writing Batch 4 (Interaction & Capstone)
**Goal**: ~13,000 words total.

- [ ] T015 [US3] Rewrite `content/modules/07-conversational/index.md`
- [ ] T016 [US3] Rewrite `content/modules/07-conversational/01-llm-integration.md` (Deep dive: STT/TTS, RAG, Function Calling)
- [ ] T017 [US3] Rewrite `content/modules/08-capstone/index.md`
- [ ] T018 [US3] Rewrite `content/modules/08-capstone/01-project-spec.md` (Full detailed project guide)

## Phase 6: Polish
- [ ] T019 Run `npm run validate-content`
- [ ] T020 Run `npm run build`
