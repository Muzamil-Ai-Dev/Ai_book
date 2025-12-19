# Quickstart: Implementing Visual Redesign

## Setup

1. **Verify Docusaurus Path**: Ensure you are in `apps/web`.
2. **Install Dependencies**: No new external frameworks expected, but if motion is needed: `npm install framer-motion`.

## Implementation Order

### 1. Structural Changes
- Update `docusaurus.config.ts`:
  - Change `docs.routeBasePath` to `/modules`.
  - Update `navbar` and `footer` configuration.
- Rename `modules/index.md` if necessary to avoid collision with the new homepage.

### 2. Design System
- Update `src/css/custom.css` with the variables defined in `data-model.md`.
- Add Google Font imports (Inter, JetBrains Mono) to `docusaurus.config.ts`.

### 3. Landing Page
- Create `src/pages/index.tsx`.
- Implement Hero and Feature sections using Tailwind-like CSS or Infima classes.

### 4. Reading Experience
- Swizzle `DocItem` if needed to adjust the reading area layout.
- Apply `max-width` and typography overrides in `custom.css`.

## Verification

- Run `npm run start` to check local dev.
- Run `npm run build` to ensure no breaks in routing or SSR.
- Check accessibility using Lighthouse.
