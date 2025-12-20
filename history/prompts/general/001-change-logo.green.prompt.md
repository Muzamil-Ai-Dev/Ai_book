---
id: 1
title: Change website logo and make it circular
stage: green
date: 2025-12-20
surface: agent
model: gpt-4o
feature: logo-update
branch: master
user: muzamil
command: change logo
labels: ["ui", "logo", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - apps/web/static/img/logo.jpeg
  - apps/web/docusaurus.config.ts
  - apps/web/src/css/custom.css
tests:
  - none
---

## PROMPT_TEXT
hello this is my project every thing is look good in this website but the logo of wesite is not looking good here is a path of imge of logo which i want you to chage with the current logo "C:\Users\muzamil\Downloads\WhatsApp Image 2025-12-20 at 4.09.34 PM.jpeg" one more thing in this logo there is also a text so dont add it only add the logo which is in circle    and dont try to change anything

## RESPONSE_TEXT
I have updated the website logo. I copied the new image to `apps/web/static/img/logo.jpeg`, updated the configuration in `docusaurus.config.ts`, and added CSS styles in `custom.css` to ensure the logo is circular and crops out any text outside the circular area.

## OUTCOME
The logo has been successfully changed and styled as requested.
