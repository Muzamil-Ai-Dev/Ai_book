# Data Model: Visual Design System

## Theme Variables (CSS)

### Color Palette (Futuristic Dark)

| Variable | Value | Description |
|----------|-------|-------------|
| `--ifm-color-primary` | `#00d1b2` | Teal/Cyan for primary actions and branding |
| `--ifm-background-color` | `#0b0e14` | Main background for pages |
| `--ifm-navbar-background-color` | `#11141b` | Distinct background for the navbar |
| `--ifm-footer-background-color` | `#0a0c10` | Darker footer background |
| `--text-color-base` | `#e0e6ed` | High-contrast text for reading |
| `--text-color-secondary` | `#94a3b8` | Lower contrast for metadata/subheadings |

### Typography

| Category | Font Family |
|----------|-------------|
| Body (Sans) | `'Inter', system-ui, -apple-system, sans-serif` |
| Code (Mono) | `'JetBrains Mono', 'Fira Code', monospace` |
| Headings | `'Inter', sans-serif` |

### Layout Constraints

| Constraint | Value | Description |
|------------|-------|-------------|
| Max Reading Width | `850px` | Enforced on `.markdown` container for readability |
| Base Font Size | `17px` | Slightly larger for academic comfort |
| Border Radius | `8px` | Subtle rounding for modern feel |

## Landing Page Components

- **HeroSection**: `title`, `subtitle`, `cta_link`, `cta_text`
- **FeatureSection**: `icon`, `title`, `description` (Used for "What you will learn")
- **ModuleOverview**: `title`, `description`, `link`
