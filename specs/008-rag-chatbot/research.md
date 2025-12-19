# Research: Grounded RAG Chatbot

This document outlines the research and architectural decisions for the implementation of the Grounded RAG Chatbot.

## 1. Orchestration: OpenAI ChatKit SDK (Python)

### Decision
Use the `openai-chatkit` Python SDK integrated with a FastAPI backend.

### Rationale
- Provides a structured way to manage chat threads, items, and streaming responses.
- Supports "Hidden Context" which is ideal for injecting RAG context (retrieved chunks) without cluttering the visible chat history.
- Built-in SSE (Server-Sent Events) support for streaming responses, enhancing the "AI-Native" feel.

### Implementation Pattern
- Define a `MyChatKitServer` inheriting from `ChatKitServer`.
- Implement a `respond` method that uses the `Runner.run_streamed` from the Agents SDK.
- Use a `ThreadItemConverter` to inject retrieved context as `HiddenContextItem`.

## 2. Grounding: Context-7 MCP Server

### Decision
Integrate Context-7 via the Model Context Protocol (MCP) as a mandatory grounding layer.

### Rationale
- Context-7 is specifically designed for factual grounding of coding and documentation context.
- Provides a "Hard Grounding" guarantee by ensuring the LLM only answers from the provided MCP resources.
- Standardizes the interface between the AI agent and external knowledge bases.

### Implementation Pattern
- The FastAPI backend will act as an MCP client or proxy to a Context-7 server instance.
- Before generating a response, retrieved textbook chunks will be passed through Context-7 to verify relevance and safety (no external knowledge).
- If Context-7 returns a low-confidence or "not found" status, the system will trigger a refusal response.

## 3. Ingestion: Deterministic Markdown Chunking

### Decision
Implement a deterministic chunking strategy using Docusaurus heading hierarchies.

### Rationale
- Docusaurus content is naturally structured by headings (`H1`, `H2`, `H3`).
- Determinism ensures that the same content always maps to the same IDs/embeddings, making updates more efficient.
- Preserving the "Heading Path" (e.g., `Module 01 > Intro > Embodied AI`) provides superior context for retrieval compared to fixed-size sliding windows.

### Implementation Pattern
- Use `markdown-it` or `BeautifulSoup` to parse the Docusaurus Markdown/MDX.
- Split files by heading boundaries.
- Assign metadata: `module` (folder name), `chapter` (file name), `heading` (current section), `file_path`.
- Store chunks in Qdrant with a unique ID derived from the `file_path` and `heading`.

## 4. LLM: Gemini via ChatKit

### Decision
Use Gemini (via Gemini API key) as the generative model, orchestrated through the ChatKit/OpenAI Agents flow.

### Rationale
- Gemini provides high-quality reasoning and supports long context windows if needed.
- Using ChatKit as an abstraction layer allows us to switch models if necessary while maintaining the same frontend/UI interaction pattern.

### Alternatives Considered
- **Direct OpenAI API**: Rejected to maintain the flexibility of using Gemini while using OpenAI's orchestration primitives (ChatKit).
- **LangChain/LlamaIndex**: While useful for utilities, we prefer a more direct implementation using ChatKit to ensure a lightweight and custom-tailored grounding flow.
