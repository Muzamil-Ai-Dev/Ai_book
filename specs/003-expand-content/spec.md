# Feature Specification: Expand Textbook Content (Text Only)

**Feature Branch**: `003-expand-content`
**Created**: 2025-12-16
**Status**: Draft
**Input**: Expand the textbook content to approximately 50,000 words total. Rewrite all existing skeleton chapters into full, detailed, pedagogical text. Do NOT include any images or image links. Focus on depth, examples, and clear explanations.

## User Scenarios & Testing

### User Story 1 - Deep Reading (Priority: P1)
A learner reads the "Introduction to Physical AI" module and gains a comprehensive understanding of the field without needing external references or diagrams.
**Why this priority**: The current content is skeletal. We need depth to make it a real textbook.
**Independent Test**: Read the generated chapter. It should be at least 2000-3000 words long and contain at least 3 detailed concrete examples per concept.
**Acceptance Scenarios**:
1. **Given** the Intro module, **When** read, **Then** it covers definitions, history, ethical considerations, and future outlook in depth.
2. **Given** the "No Images" constraint, **When** a complex concept is explained, **Then** it uses rich analogies or code blocks instead of diagrams.

### User Story 2 - Technical Depth in ROS 2 (Priority: P1)
A learner follows the ROS 2 module and understands the underlying architecture (DDS, RMW) through detailed text and code analysis.
**Acceptance Scenarios**:
1. **Given** the Node explanation, **When** read, **Then** it explains the lifecycle, executor models, and callback groups in detail.

### User Story 3 - Full Book Expansion (Priority: P2)
All 8 modules are expanded to their target length.
**Target Word Counts**:
- Intro: 3,000 words
- ROS 2: 8,000 words
- Gazebo/Unity: 5,000 words
- Isaac: 5,000 words
- VLA: 8,000 words
- Humanoid: 8,000 words
- Conversational: 5,000 words
- Capstone: 8,000 words
**Total**: ~50,000 words.

## Requirements

### Functional Requirements
- **FR-001**: All existing Markdown files in `content/modules/` MUST be rewritten.
- **FR-002**: NO images, `![]()`, or `<img />` tags are permitted. All visual concepts must be described via text or ASCII art/Mermaid (if safe, but text preferred).
- **FR-003**: Each chapter MUST maintain the 5-section structure (Objectives, Concepts, Visual descriptions (text only), Exercises, Capstone Prep).
- **FR-004**: Code examples MUST be extensive, commented, and self-contained.
- **FR-005**: "Visual Descriptions" section MUST be renamed to "Conceptual Visualization" and contain descriptive text painting a mental image, rather than image links.

### Key Entities
- **Chapter Text**: The body content.
- **Code Block**: Syntactically highlighted code (Python/C++/Bash).

## Success Criteria
- **SC-001**: Total word count across `content/modules` exceeds 40,000 words (MVP) to 50,000 words.
- **SC-002**: `npm run build` passes with ZERO image-related warnings or errors.
- **SC-003**: Every chapter has "Deep" content (no "Lorem Ipsum" or "Coming Soon" or shallow bullet points).