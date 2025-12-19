# Research: Content Structure & Best Practices

**Decision**: Use Standard Docusaurus Versioning and Sidebars
**Rationale**: Docusaurus provides built-in support for document versioning and sidebar generation. Using these standard features ensures maintainability and easy navigation for learners. We will structure content using numeric prefixes (e.g., `01-intro`) to ensure correct ordering without manual sidebar configuration where possible.

**Decision**: Local Asset Management via `static/img`
**Rationale**: To comply with FR-006 (No external hotlinking) and ensure offline renderability, all assets will be stored in `apps/web/static/img/modules/{module_name}`. This keeps assets organized by module and prevents the root directory from becoming cluttered.

**Decision**: MDX for Interactive Elements
**Rationale**: We will leverage MDX (Markdown + JSX) to include interactive elements like quizzes or collapsible "Solution" blocks for exercises, enhancing the pedagogical value.

**Decision**: Custom Content Validation Script
**Rationale**: Docusaurus checks broken links, but not *structure* (e.g., missing "Learning Objectives"). To strictly enforce the "Content as First-Class Artifact" principle and SC-002, we will implement `scripts/validate-content.js`. This script will parse ASTs to verify required sections and valid frontmatter before the build step.

**Alternatives Considered**:
- **Remote CMS (Strapi/Contentful)**: Rejected because "Content as First-Class Artifact" principle mandates git-based versioning and the spec requires Markdown.
- **Single Monolithic Markdown File**: Rejected due to the scale of the book; modularity is essential for maintainability.

## Resolved Clarifications

- **Hardware Requirements**: Resolved in Spec Update. We will explicitly mark modules requiring NVIDIA GPUs.
- **Asset Pathing**: Resolved to use standard Docusaurus static folder patterns (`![](/img/...)`).
