# Feature Specification: Grounded RAG Chatbot

**Feature Branch**: `008-rag-chatbot`  
**Created**: 2025-12-19  
**Status**: Draft  
**Input**: User description: "Create a specification for implementing a Retrieval-Augmented Generation (RAG) chatbot for the AI-Native Textbook for Physical AI & Humanoid Robotics. This chatbot is a core AI-Native feature and must demonstrate grounded, non-hallucinating intelligence. ──────────────────────────────── PURPOSE ──────────────────────────────── Enable users to ask questions and receive answers strictly grounded in the textbook’s Markdown content. The chatbot must: • Answer ONLY from book content • Refuse to answer when information is missing • Expose retrieved sources for transparency • Demonstrate AI-Native learning behavior ──────────────────────────────── ARCHITECTURE REQUIREMENTS ──────────────────────────────── • Backend: FastAPI • Orchestration: OpenAI ChatKit SDK • LLM for response generation: Gemini (via Gemini API key) • Retrieval: Embeddings + vector database • Grounding: Context-7 MCP server (mandatory) • Content source: Docusaurus Markdown ──────────────────────────────── CONTENT SOURCE ──────────────────────────────── • Canonical source: content/modules/ • Each Markdown chapter is a retrievable unit • Content must be chunked with metadata: - module - chapter - heading - file path ──────────────────────────────── CORE COMPONENTS ──────────────────────────────── 1. INGESTION PIPELINE • Read Markdown files from content/modules/ • Chunk content deterministically • Generate embeddings • Store embeddings in vector DB • Preserve source metadata Endpoint: POST /ingest ──────────────────────────────── 2. RETRIEVAL + GROUNDING • Retrieve top-K relevant chunks • Pass retrieved context through Context-7 MCP • Apply relevance filtering • If confidence is low → refuse Context-7 must ensure: • No external knowledge • No speculative answers • Hard grounding to retrieved text ──────────────────────────────── 3. RESPONSE GENERATION • OpenAI ChatKit is used to orchestrate the flow: - user question - retrieved context - refusal rules • Gemini LLM is used ONLY for final response generation • Gemini must not receive any information outside retrieved context ──────────────────────────────── 4. QUERY ENDPOINT Endpoint: POST /query Behavior: • Accepts user question • Retrieves relevant content • Applies Context-7 grounding • Uses ChatKit + Gemini to generate response • Returns: - answer - sources used - refusal reason (if applicable) ──────────────────────────────── NON-FUNCTIONAL REQUIREMENTS ──────────────────────────────── • Deterministic chunking • No hardcoded secrets • Environment variables only • Clear logging for: - retrieved chunks - grounding decisions • Simple JSON responses (no UI logic) ──────────────────────────────── VERIFICATION CRITERIA ──────────────────────────────── • Questions answerable from the book → correct grounded response • Questions not in the book → refusal • Logs show retrieved source files • Gemini responses never exceed provided context ──────────────────────────────── OUTCOME ──────────────────────────────── A production-ready, AI-Native RAG chatbot that proves the textbook is interactive, grounded, and non-hallucinating. This system is evaluated as a core hackathon deliverable."

## User Scenarios & Testing _(mandatory)_



### User Story 1 - Answering from Textbook Content (Priority: P1)



As a student reading the AI-Native Textbook, I want to ask questions about robotics concepts so that I can get immediate, accurate answers that are directly derived from the book's material.



**Why this priority**: This is the core value proposition of the RAG chatbot - providing grounded information from the textbook.



**Independent Test**: Can be tested by asking a question that is clearly covered in the textbook modules (e.g., "What is a VLA model?") and verifying the response matches the book content.



**Acceptance Scenarios**:



1. **Given** the textbook content has been ingested, **When** a user asks "Explain ROS 2 nodes", **Then** the system returns an answer based on the ROS 2 module and lists the source chapter.

2. **Given** a user question, **When** the system retrieves relevant chunks, **Then** it must only use those chunks to generate the final response.



---



### User Story 2 - Refusing Out-of-Scope Questions (Priority: P2)



As a user, I want the chatbot to admit when it doesn't know an answer rather than making things up, so that I can trust the information it provides is strictly from the textbook.



**Why this priority**: Crucial for the "non-hallucinating" and "grounded" requirement.



**Independent Test**: Can be tested by asking a question about a topic not covered in the book (e.g., "How do I bake a cake?") and verifying the system returns a polite refusal.



**Acceptance Scenarios**:



1. **Given** a question about a topic absent from the modules, **When** the system processes the query, **Then** it returns a refusal message stating the information is not in the book.

2. **Given** a query that yields low-confidence retrieval results, **When** the system applies its grounding logic, **Then** it must refuse to answer.



---



### User Story 3 - Source Transparency (Priority: P3)







As a researcher, I want to see exactly where the chatbot found its information so that I can verify the context and read the original text in the textbook.







**Why this priority**: Enhances trust and provides a path for further reading.







**Independent Test**: Can be tested by checking the source references in the API response for any successful query.







**Acceptance Scenarios**:







1. **Given** a successful answer, **When** the response is returned, **Then** it includes metadata for the source (module, chapter, path).



2. **Given** multiple sources are used, **When** the response is generated, **Then** all contributing sources are listed.







---







### User Story 4 - AI-Native Learning Behavior (Priority: P4)







As a student, I want to ask follow-up questions about a topic so that I can explore complex concepts through a natural dialogue that maintains the context of our previous conversation.







**Why this priority**: Demonstrates "AI-Native learning behavior" through contextual continuity and iterative exploration.







**Independent Test**: Can be tested by asking "What is ROS 2?" followed by "Give me an example of its usage" and verifying the second answer relates to ROS 2 without explicitly naming it.







**Acceptance Scenarios**:







1. **Given** a previous interaction in the same session, **When** a user asks a follow-up question, **Then** the system uses the conversation history to inform the retrieval and generation.



2. **Given** a multi-turn dialogue, **When** the system generates an answer, **Then** it remains strictly grounded in the textbook content relative to the entire conversation context.







### Edge Cases







- **Ambiguous Queries**: What happens when a user query matches multiple conflicting sections of the book?



- **Empty Content**: How does the system handle an ingestion request when no Markdown files are found in the target directory?



- **Very Long Queries**: How does the system handle queries that exceed standard token limits for retrieval or generation?



- **Non-Text Content**: How does the system handle Markdown files that contain only images or code blocks without explanatory text?



- **Conversation Drift**: How does the system handle follow-up questions that lead away from the textbook's scope?







## Constraints & Assumptions







### Architectural Constraints







- **Backend**: System must be implemented using FastAPI.



- **Orchestration**: System must use OpenAI ChatKit SDK for workflow orchestration.



- **LLM**: Gemini must be used for final response generation.



- **Grounding**: Context-7 MCP server is mandatory for ensuring responses are grounded.



- **Content Source**: Canonical source is `content/modules/` in Docusaurus Markdown format.







## Requirements _(mandatory)_







### Functional Requirements







- **FR-001**: System MUST ingest Markdown files from the textbook content directory and chunk them deterministically.



- **FR-002**: System MUST generate and store semantic representations (embeddings) with mandatory metadata: `module`, `chapter`, `heading`, and `file path`.



- **FR-003**: System MUST answer user queries ONLY using information retrieved from the ingested textbook content.



- **FR-004**: System MUST refuse to answer if the required information is missing or the retrieval confidence is low.



- **FR-005**: System MUST provide the specific source files used for every answer generated.



- **FR-006**: System MUST enforce strict grounding via a mandatory cross-verification step between the generated response and the retrieved context.



- **FR-007**: System MUST maintain session-based conversation history to support contextual follow-up questions.



- **FR-008**: System MUST provide an interface to process and update the textbook content (`POST /ingest`).



- **FR-009**: System MUST provide an interface that accepts user questions and returns grounded answers with sources (`POST /query`).







### Key Entities _(include if feature involves data)_







- **Content Chunk**: A segment of a textbook file with mandatory metadata: `module`, `chapter`, `heading`, `file path`.



- **Semantic Representation**: A mathematical representation of a Content Chunk used for relevance matching.



- **Conversation Context**: A session-based store of user questions and grounded answers used for multi-turn dialogue.



- **Query Response**: A structured object containing the generated answer, source references (`module`, `chapter`, `path`), and any refusal reasoning.







## Success Criteria _(mandatory)_



### Measurable Outcomes



- **SC-001**: 100% of responses to questions answerable from the textbook must contain information present in the source modules.

- **SC-002**: 0% hallucination rate; system must never provide facts not found in the textbook or provided context.

- **SC-003**: System must return a refusal for 100% of queries that are completely unrelated to the textbook content.

- **SC-004**: Every non-refusal response MUST include at least one valid source reference from the textbook.

- **SC-005**: Query processing must be performant enough to support an interactive user experience.
