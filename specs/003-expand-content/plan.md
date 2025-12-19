# Implementation Plan: Expand Textbook Content (Text Only)

**Branch**: `003-expand-content` | **Date**: 2025-12-16 | **Spec**: [specs/003-expand-content/spec.md](../spec.md)

## Summary
Systematically rewrite all 8 modules of the textbook to achieve a total word count of ~50,000 words. This pass will strictly remove all image references to ensure build stability and focus purely on textual pedagogical depth.

## Technical Context
**Constraint**: NO IMAGES. All visual explanations must be converted to rich text descriptions or code-based representations.
**Validation**: Update `scripts/validate-content.js` to strictly FORBID any image syntax (`![]`, `<img`).
**Tools**: LLM generation for long-form content.

## Constitution Check
- [x] **Content as First-Class Artifact**: This is the core of this feature.
- [x] **Verifiable Outputs**: Word count analysis and build verification.
- [x] **Pedagogical Clarity**: Shifting from visual to text-based conceptualization requires higher clarity in writing.

## Project Structure
No new directories. Working entirely within `content/modules`.

## Execution Phases

### Phase 1: Tooling Update
- Update `scripts/validate-content.js` to ban images and accept "Conceptual Visualization" as a section header.

### Phase 2: Writing (Iterative)
- **Batch 1**: Modules 01 & 02 (Intro, ROS 2)
- **Batch 2**: Modules 03 & 04 (Simulators)
- **Batch 3**: Modules 05 & 06 (AI & Humanoids)
- **Batch 4**: Modules 07 & 08 (Conversation & Capstone)

### Phase 3: Validation
- Run build.
- Count words.