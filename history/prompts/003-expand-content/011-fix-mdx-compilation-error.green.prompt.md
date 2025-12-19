---
id: 011
title: fix-mdx-compilation-error
stage: green
date: 2025-12-19
surface: agent
model: gemini-2.0-flash-exp
feature: 003-expand-content
branch: 003-expand-content
user: Muzamil
command: replace
labels: ["bugfix", "mdx", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - content/modules/05-vla/01-transformers-robotics.md
tests:
  - manual verification of file content
---

# Fix MDX Compilation Error

## Prompt

we were working on this step (003-expand-content*)  and you were implementing requirement on C:\mmuzamil\spec-kit-plus-projects\Ai_book\content\modules\06-humanoid but error comes              PS C:\mmuzamil\spec-kit-plus-projects\Ai_book> cd .\apps\   
PS C:\mmuzamil\spec-kit-plus-projects\Ai_book\apps> cd .\web\    
PS C:\mmuzamil\spec-kit-plus-projects\Ai_book\apps\web> npm run start

> apps-web@0.0.0 prestart
> node ../../scripts/sync-content.js

✅ Synced content from C:\mmuzamil\spec-kit-plus-projects\Ai_book\content\modules to C:\mmuzamil\spec-kit-plus-projects\Ai_book\apps\web\docs

> apps-web@0.0.0 start
> docusaurus start

[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/Ai_book/

× Client
  Compiled with some errors in 3.92s

Error: MDX compilation failed for file "C:\mmuzamil\spec-kit-plus-projects\Ai_book\apps\web\docs\05-vla\01-transformers-robotics.md"
Cause: Unexpected character `3` (U+0033) after name, expected a character that can start an attribute name, such as a letter, `$`, or `_`; whitespace before attributes; or the end of the tag
Details:
{
  "column": 32,
  "file": "",
  "message": "Unexpected character `3` (U+0033) after name, expected a character that can start an attribute name, such as a letter, `$`, or `_`; whitespace before attributes; or the end of the tag",
  "line": 82,
  "name": "82:32",
  "place": {
    "_bufferIndex": 31,
    "_index": 2,
    "line": 82,
    "column": 32,
    "offset": 3817
  },
  "reason": "Unexpected character `3` (U+0033) after name, expected a character that can start an attribute name, such as a letter, `$`, or `_`; whitespace before attributes; or the end of the tag",
  "ruleId": "unexpected-character",
  "source": "micromark-extension-mdx-jsx",
  "url": "https://github.com/micromark/micromark-extension-mdx-jsx#unexpected-character-at-expected-expect"
}

* Client █████████████████████████ done (99%) plugins
 webpack-dev-middleware

client (webpack 5.103.0) compiled with 1 error
<w> [webpack.cache.PackFileCacheStrategy] Skipped not serializable cache item 'Compilation/modules|C:\mmuzamil\spec-kit-plus-projects\Ai_book\node_modules\@docusaurus\mdx-loader\lib\index.js??ruleSet[1].rules[8].use[0]!C:\mmuzamil\spec-kit-plus-projects\Ai_book\apps\web\docs\05-vla\01-transformers-robotics.md': No serializer registered for VFileMessage
<w> while serializing webpack/lib/cache/PackFileCacheStrategy.PackContentItems -> webpack/lib/NormalModule -> webpack/lib/ModuleBuildError -> Error -> VFileMessage

## Response

Fixed the MDX syntax error in `content/modules/05-vla/01-transformers-robotics.md` by escaping `<Token ...>` sequences (e.g., `\<Token 30100\>`) that were being incorrectly parsed as JSX tags by Docusaurus.

```