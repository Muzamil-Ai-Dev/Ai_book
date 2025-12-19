# Research: AI-Native Textbook Visual & UX Redesign

## Docusaurus Customization Patterns

### Custom Landing Page
- **Decision**: Change `docs.routeBasePath` from `/` to `/modules` in `docusaurus.config.ts`.
- **Rationale**: This frees up the root `/` path for a custom React-based landing page at `src/pages/index.tsx`.
- **Alternatives**: Using a docs page as a homepage (current state) - rejected because it lacks the marketing/startup aesthetic flexibility required by the spec.

### Theme Overrides (Swizzling)
- **Decision**: Use `npm run swizzle` to wrap or replace `DocItem`, `DocSidebar`, and `Footer` if CSS alone is insufficient.
- **Rationale**: Swizzling allows for structural changes to the layout that CSS cannot achieve (e.g., adding custom navigation elements or changing the sidebar hierarchy display).
- **Alternatives**: CSS-only overrides - rejected for structural goals like "clear separation between sidebar and content" and "sticky progress indicators".

### Design System (Infima + Custom CSS)
- **Decision**: Define a "Futuristic Academic" palette using CSS variables in `src/css/custom.css`.
- **Rationale**: Infima is the underlying framework. Overriding its `--ifm-*` variables ensures consistency across all Docusaurus components.
- **Color Palette Ideas**: 
  - Dark-first aesthetic.
  - Primary: Cyan/Teal (#00f5d4 or similar) for an AI/Futuristic feel.
  - Background: Deep slate/near-black (#0b0e14).
  - Typography: Inter or Roboto for Sans-serif; JetBrains Mono or Fira Code for Monospace.

### Navigation Hierarchy
- **Decision**: Update `sidebars.ts` to ensure consistent grouping and use the "Modules" label in the navbar.
- **Rationale**: Aligns with the spec's goal of "clear module hierarchy".

## Technical Unknowns Resolved

| Unknown | Finding |
|---------|---------|
| Overriding Landing Page | Requires moving docs to a sub-route and creating `index.tsx` in `src/pages`. |
| Logo Implementation | Can use SVG for the logo and reference it in `docusaurus.config.ts`. |
| Typography Fonts | Can be imported via Google Fonts in `docusaurus.config.ts` stylesheets. |
| Optimal Line Length | Can be enforced via `max-width` on the `.markdown` or `container` class in `custom.css`. |
