# Design Tokens & CSS Data Model

## Core Variables (Neon/Glow)

| Variable | Value (Dark Mode) | Purpose |
| :--- | :--- | :--- |
| `--ifm-color-primary` | `#00f2ff` (Neon Cyan) | Main brand color |
| `--glow-primary` | `0 0 15px rgba(0, 242, 255, 0.5)` | Box shadow for primary elements |
| `--glass-bg` | `rgba(15, 23, 42, 0.7)` | Backdrop for glassmorphism |
| `--glass-border` | `rgba(255, 255, 255, 0.1)` | Subtle border for glass effects |

## Motion System Classes

| Class | Animation | Usage |
| :--- | :--- | :--- |
| `.animate-in` | Fade-in + Slide-up (20px) | Section entrance |
| `.hover-glow` | Transition: scale(1.02) + glow increase | Cards and buttons |
| `.float` | Infinite floating Y-axis | Background blobs |

## Component States

### Glass Card
- **Normal**: `backdrop-filter: blur(12px)`, `border: 1px solid var(--glass-border)`
- **Hover**: `border-color: var(--ifm-color-primary)`, `box-shadow: var(--glow-primary)`

### Custom Scrollbar
- **Width**: `6px`
- **Track**: `transparent`
- **Thumb**: `var(--ifm-color-primary)` with `border-radius: 10px`