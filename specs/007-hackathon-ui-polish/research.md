# Research: Hackathon-Winning UI & UX Polish

## 1. Glassmorphism Implementation
**Decision**: Use pure CSS `backdrop-filter` with semi-transparent background variables.
**Rationale**: Native browser support is strong, and it avoids heavy JS libraries.
**Implementation**:
- Target `.navbar` and custom `.glass-card` classes.
- Use `-webkit-backdrop-filter` for Safari support.
- Background: `rgba(var(--bg-rgb), 0.7)`.

## 2. Motion System
**Decision**: Use `framer-motion` for complex interactions and CSS `IntersectionObserver` for simple entrance animations.
**Rationale**: `framer-motion` provides the "high-fidelity" feel requested (spring physics), while standard CSS animations handle performance for basic scrolls.
**Alternatives Considered**: `AOS` (rejected due to less control in React), `React Spring` (rejected as Framer Motion is more declarative for this scale).

## 3. Bespoke Code Blocks
**Decision**: Swizzle `@docusaurus/theme-classic CodeBlock` to add a custom header.
**Rationale**: Swizzling is the canonical Docusaurus way to modify internal components safely.
**Features**:
- macOS-style traffic light buttons (visual only).
- Move existing Copy button into the new header.
- Add language label.

## 4. Performance & Hardware Acceleration
**Decision**: Animate ONLY `transform` and `opacity`.
**Rationale**: Ensures 60fps on mobile and low-end devices by avoiding layout reflows (avoiding `top`, `left`, `width`, `height` animations).

## 5. Background Glow Blobs
**Decision**: 2-3 absolute-positioned `div`s with `filter: blur(80px)` and infinite `transform: translate()` animations.
**Rationale**: Extremely lightweight compared to SVG or Canvas while achieving the same "futuristic" aesthetic.