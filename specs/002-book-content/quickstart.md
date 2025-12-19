# Quickstart: Writing Book Content

## prerequisites

- Node.js & NPM (for Docusaurus preview)
- Git
- VS Code (recommended)

## 1. Setup

```bash
# Clone repository
git clone <repo-url>
cd Ai_book

# Install dependencies
cd apps/web
npm install
```

## 2. Create a New Chapter

1. Navigate to `content/modules/{module-name}/`.
2. Create a new markdown file: `XX-topic-name.md`.
3. Add the required frontmatter:

   ```markdown
   ---
   id: topic-name
   title: Topic Name
   sidebar_label: Topic
   description: Learn about Topic Name.
   ---
   ```

## 3. Writing Guidelines

- **Structure**: Follow the 5 mandatory sections:
  1. `## Learning Objectives`
  2. `## Concept Explanations`
  3. `## Visual Descriptions`
  4. `## Hands-on Exercises`
  5. `## Capstone Prep`
- **Images**: Place images in `apps/web/static/img/modules/{module}/`. Reference them as `![](/img/modules/{module}/image.png)`.
- **Code**: Use fenced code blocks with language tags:

   ```python
   print("Hello ROS 2")
   ```

## 4. Preview

```bash
cd apps/web
npm run start
```
Open `http://localhost:3000` to see your changes live.

