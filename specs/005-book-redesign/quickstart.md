# Quickstart: Verifying Book Redesign

## Prerequisites

- Node.js >= 20.0
- npm

## Setup

1.  Install dependencies:
    ```bash
    npm install
    ```

## Running the Development Server

1.  Start the Docusaurus server:
    ```bash
    cd apps/web
    npm start
    ```

## Verification Steps

1.  **Sidebar**: Open `http://localhost:3000/`. Verify you see "Intro", "ROS 2", etc. in the left sidebar.
2.  **Navigation**: Click through modules. Verify structure is flat (Intro, Module 1, Module 2, Capstone).
3.  **Root URL**: Verify `http://localhost:3000/` loads the Intro content.
4.  **Auxiliary Pages**:
    *   Click "Glossary" in top Navbar. Verify it loads.
    *   Click "Notation" in top Navbar. Verify it loads and Math is rendered.
5.  **Chapter Structure**: Check `apps/web/modules/02-ros2/01-nodes.md` (or similar). Verify it has required sections.
