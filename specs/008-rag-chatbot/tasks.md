# Tasks: Grounded RAG Chatbot

**Input**: Design documents from `/specs/008-rag-chatbot/`  
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api.yaml

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create `apps/api` directory structure per implementation plan
- [x] T002 Initialize Python project with `pyproject.toml` or `requirements.txt` in `apps/api/`
- [x] T003 [P] Configure `.env` with Gemini, OpenAI, and Qdrant placeholders in `apps/api/.env`
- [x] T004 [P] Setup `pytest` configuration in `apps/api/tests/`
- [x] T005 [P] Implement environment variable validation script in `apps/api/src/utils/config.py`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [x] T006 [P] Implement Pydantic models for `ContentChunk` and `SourceMetadata` in `apps/api/src/models/content.py`
- [x] T007 [P] Implement vector DB client (Qdrant) wrapper in `apps/api/src/utils/vector_db.py`
- [x] T008 Implement deterministic chunking utility in `apps/api/src/utils/chunking.py`
- [x] T009 [P] Implement basic FastAPI app with ChatKit server initialization in `apps/api/src/main.py`
- [x] T010 [P] Setup basic logging for retrieval and grounding in `apps/api/src/utils/logging.py`
- [x] T011 [P] Implement Context-7 MCP connection health check and wrapper in `apps/api/src/utils/grounding.py`

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Answering from Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Answer questions based on ingested Docusaurus Markdown.

**Independent Test**: Ask "What is ROS 2 nodes?" and receive a correct grounded answer.

### Implementation for User Story 1

- [x] T012 [US1] Implement Ingestion Service in `apps/api/src/services/ingestion.py` (reads `content/modules/`)
- [x] T013 [US1] Implement `POST /ingest` endpoint in `apps/api/src/api/ingest.py`
- [x] T014 [US1] Implement RAG Retrieval logic in `apps/api/src/services/retrieval.py`
- [x] T015 [US1] Implement Grounding Logic with Context-7 MCP placeholder in `apps/api/src/utils/grounding.py`
- [x] T016 [US1] Implement `POST /query` endpoint for basic Q&A in `apps/api/src/api/query.py`
- [x] T017 [US1] Define grounding system prompt with refusal rules in `apps/api/src/services/chat_service.py`
- [x] T018 [US1] Integrate Gemini via ChatKit using `HiddenContextItem` for RAG chunks in `apps/api/src/services/chat_service.py`

**Checkpoint**: User Story 1 functional (Answers from book content).

---

## Phase 4: User Story 2 - Refusing Out-of-Scope Questions (Priority: P2)

**Goal**: Admitting "I don't know" for non-textbook topics.

**Independent Test**: Ask "How to bake a cake?" and receive a polite refusal.

### Implementation for User Story 2

- [x] T019 [US2] Implement relevance thresholding in `apps/api/src/services/retrieval.py`
- [x] T020 [US2] Implement Context-7 hard grounding check in `apps/api/src/utils/grounding.py`
- [x] T021 [US2] Add refusal reasoning to `ChatResponse` model in `apps/api/src/models/response.py`
- [x] T022 [US2] Update `chat_service.py` to trigger refusal based on grounding results.

**Checkpoint**: User Story 2 functional (No hallucinations).

---

## Phase 5: User Story 3 - Source Transparency (Priority: P3)

**Goal**: Show exactly where the answer came from (module, chapter, path).

**Independent Test**: Check `sources` array in API response for module/chapter metadata.

### Implementation for User Story 3

- [x] T023 [US3] Ensure `module`, `chapter`, and `path` are captured during ingestion in `ingestion.py`
- [x] T024 [US3] Update `SourceMetadata` entity to match required fields in `models/content.py`
- [x] T025 [US3] Ensure `retrieval.py` returns full metadata with chunks.
- [x] T026 [US3] Update `POST /query` response to include detailed source list in `api/query.py`.

**Checkpoint**: User Story 3 functional (Source attribution).

---

## Phase 6: User Story 4 - AI-Native Learning (Priority: P4)

**Goal**: Maintain conversation context for follow-up questions.

**Independent Test**: Ask follow-up question "Give me an example" after a ROS 2 question.

### Implementation for User Story 4

- [x] T027 [US4] Implement `ConversationContext` entity in `apps/api/src/models/session.py`
- [x] T028 [US4] Update ChatKit server to manage sessions in `src/main.py`
- [x] T029 [US4] Update `retrieval.py` to use conversation history for query expansion/re-ranking.
- [x] T030 [US4] Implement context-aware response generation in `chat_service.py`.

**Checkpoint**: User Story 4 functional (Contextual dialogue).

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final verification and cleanup.

- [x] T031 [P] Security check: Verify no keys in logs in `apps/api/src/utils/logging.py`
- [x] T032 Performance profiling for query response time (target < 5s).
- [x] T033 [P] Final documentation update in `apps/api/README.md`
- [x] T034 Run `quickstart.md` validation.

---

## Dependencies & Execution Order

### Phase Dependencies
1. **Setup (Phase 1)** -> **Foundational (Phase 2)** (BLOCKS ALL STORIES)
2. **User Story 1 (P1)** -> **User Story 3 (P3)** (Transparency depends on ingestion metadata)
3. **User Story 1 (P1)** -> **User Story 2 (P2)** (Refusal depends on retrieval success)
4. **User Story 1 (P1)** -> **User Story 4 (P4)** (Context depends on basic Q&A working)

### Parallel Opportunities
- T003, T004, T005, T006, T007, T011 can run in parallel.
- Once US1 foundational endpoints are in, US2 and US3 implementation can overlap slightly.

---

## Implementation Strategy

### MVP First (User Story 1 Only)
Complete Phases 1, 2, and 3. This gives you a working RAG bot that answers from the book.

### Incremental Delivery
1. Foundation -> Ingestion + Query (US1) -> Transparency (US3) -> Refusal (US2) -> Context (US4).