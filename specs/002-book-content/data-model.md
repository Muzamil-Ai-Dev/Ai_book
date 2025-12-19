# Data Model: Content Hierarchy

This "data model" represents the file and folder structure for the textbook content.

## Entity: Module
A top-level directory representing a major topic.

| Field | Type | Description |
|---|---|---|
| ID | string | Folder name (e.g., `01-intro`) |
| Title | string | Human-readable title (from `_category_.json` or index.md) |
| Order | integer | Sequence number (01-08) |

## Entity: Chapter
A Markdown file representing a specific lesson.

| Field | Type | Description |
|---|---|---|
| ID | string | File name (e.g., `01-concepts.md`) |
| Frontmatter | YAML | Metadata (title, sidebar_label, description) |
| Content | MDX | The pedagogical text |

## Directory Structure Map

```text
content/modules/
├── 01-intro/
│   ├── _category_.json          # Sidebar metadata
│   ├── 01-embodied-ai.md
│   ├── 02-why-physical.md
│   └── index.md                 # Module overview
├── 02-ros2/
│   ├── _category_.json
│   ├── 01-architecture.md
│   ├── 02-nodes-topics.md
│   ├── 03-services-actions.md
│   └── index.md
├── 03-gazebo-unity/
│   ├── _category_.json
│   └── ...
├── 04-isaac/
│   ├── _category_.json
│   └── ...
├── 05-vla/
│   ├── _category_.json
│   └── ...
├── 06-humanoid/
│   ├── _category_.json
│   └── ...
├── 07-conversational/
│   ├── _category_.json
│   └── ...
└── 08-capstone/
    ├── _category_.json
    └── ...
```

## Contracts: Frontmatter Schema

Every `.md` or `.mdx` file MUST adhere to this frontmatter schema:

```yaml
---
id: <unique-id>
title: <Title of the Chapter>
sidebar_label: <Short Sidebar Title>
description: <Brief summary for SEO and previews>
hide_table_of_contents: false
---
```
