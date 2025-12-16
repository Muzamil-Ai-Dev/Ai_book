# Tasks: Book Platform Infrastructure

**Feature**: Book Platform Infrastructure **Branch**: `001-book-platform` **Spec**:
[specs/001-book-platform/spec.md](./spec.md)

## Phase 1: Setup

_Goal: Initialize the monorepo structure and install base dependencies._

- [x] T001 Initialize npm project with workspaces support in `package.json`
- [x] T002 Create directory structure: `apps/web`, `apps/api`, `content/modules`
- [x] T003 Install Docusaurus v3 scaffold in `apps/web` using `npx create-docusaurus@latest`
- [x] T004 Install Prettier and configured plugins in root `package.json`
- [x] T005 Create root `.prettierrc` configuration file
- [x] T006 Add `.gitignore` to root excluding `node_modules`, `build`, and `.docusaurus`

## Phase 2: Foundational

_Goal: Establish core configuration and content seeding._

- [x] T007 Configure `apps/web/docusaurus.config.js` with project metadata (Title, Repo, Org)
- [x] T008 [P] Configure `apps/web/docusaurus.config.js` content plugin to read from
      `../../content/modules`
- [x] T009 [P] Create script `scripts/seed-content.js` to generate mandatory modules with unique
      `sidebar_position` and valid `slug` frontmatter
- [x] T010 Run seed script to populate `content/modules` with 8 placeholder chapters
- [x] T011 Verify `apps/web/sidebars.js` is set to autogenerate from content

## Phase 3: Platform Access & Navigation (User Story 1)

_Goal: Students can visit the URL and navigate the modules._

- [x] T012 [US1] Build Docusaurus locally and verify sidebar renders 8 modules
- [x] T013 [US1] Customize Homepage `apps/web/src/pages/index.js` with project branding
- [x] T014 [US1] Verify navigation links route to correct placeholder pages
- [x] T015 [US1] Run Prettier check on generated content and app code

## Phase 4: Maintainer Deployment (User Story 2)

_Goal: Maintainers can deploy updates to GitHub Pages._

- [x] T016 [US2] Add `deploy` script to `apps/web/package.json` using `docusaurus deploy`

- [x] T017 [US2] Create GitHub Actions workflow `.github/workflows/deploy.yml` for auto-deployment

      workflow deployment
- [x] T017b [US2] Configure GitHub Repo Settings (Pages source set to GitHub Actions) to enable
      workflow deployment
- [x] T018 [US2] Test local build command `npm run build` in `apps/web`

## Final Phase: Polish

- [x] T019 Clean up default Docusaurus blog and tutorial files
- [x] T020 Verify all Spec Success Criteria (SC-001 to SC-004) are met

## Dependencies

- **US1 (Access)** depends on **Foundational (Content Seeding)**
- **US2 (Deployment)** depends on **US1 (Working Build)**

## Parallel Execution Opportunities

- T008 (Config) and T009 (Seed Script) can be done in parallel.
- T013 (Homepage) can be done while T012 (Sidebar Check) is verified.
