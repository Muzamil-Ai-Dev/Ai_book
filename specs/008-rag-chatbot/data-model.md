# Data Model: Grounded RAG Chatbot

This document defines the core entities and data structures for the RAG system.

## 1. ContentChunk
Represents a deterministic segment of the textbook.

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID/String | Deterministic hash of `file_path` + `heading`. |
| `text` | String | The actual content of the chunk. |
| `module` | String | The module name (e.g., `01-intro`). |
| `chapter` | String | The filename (e.g., `01-embodied-ai.md`). |
| `heading` | String | The nearest preceding heading (e.g., `### What is VLA?`). |
| `path` | String | Relative file path in the repository. |
| `order` | Integer | Sequential index of the chunk within the file. |

## 2. SemanticRepresentation (Embedding)
The vector representation stored in the database.

| Field | Type | Description |
|-------|------|-------------|
| `chunk_id` | String | Foreign key to `ContentChunk.id`. |
| `vector` | Float[] | Embedding vector (e.g., 1536 or 768 dimensions). |
| `model` | String | The embedding model used (e.g., `text-embedding-3-small`). |

## 3. ConversationSession
Maintains state for multi-turn dialogue.

| Field | Type | Description |
|-------|------|-------------|
| `session_id` | String | Unique identifier for the user session. |
| `history` | List[Turn] | List of previous Q&A turns. |
| `last_active` | Timestamp | Used for session expiration logic. |

### Turn Structure
- `role`: `user` | `assistant`
- `content`: String
- `sources`: List[SourceMetadata] (only for assistant)

## 4. ChatResponse
The structured output from the `/query` endpoint.

| Field | Type | Description |
|-------|------|-------------|
| `answer` | String | The generated, grounded response. |
| `sources` | List[Source] | List of contributing textbook sections. |
| `refusal` | String | Optional: Reason for refusal (if not answerable). |
| `grounding_confidence` | Float | Score from Context-7 grounding check. |
