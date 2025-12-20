---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: HD
command: /sp.constitution
labels: [constitution, setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project: AI-Spec-Driven Book with Embedded RAG Chatbot

Objective:
Create a technical book using Claude Code and Spec-Kit Plus, published with Docusaurus on GitHub Pages, and embed a RAG chatbot that answers questions strictly from the book content, including user-selected text.

Core principles:
- Spec-first development
- Accuracy and no hallucination
- Clear technical writing
- Reproducibility and traceability
- Modular, maintainable architecture

Book standards:
- Audience: software engineers and AI practitioners
- Format: Docusaurus MDX
- Style: concise, technical, example-driven
- Each chapter includes objectives, examples, and summary
- Code examples must be runnable

Deployment:
- Static site via Docusaurus
- Hosted on GitHub Pages
- Repository includes specs, source, and README

RAG chatbot requirements:
- Embedded in the book UI
- Backend: FastAPI + OpenAI Agents/ChatKit
- Storage: Neon Serverless Postgres + Qdrant Cloud (Free Tier)
- Retrieval strictly limited to indexed book content
```

## Response snapshot

Updated the project constitution file with specific principles for the AI-Spec-Driven Book with Embedded RAG Chatbot project, incorporating the core principles of spec-first development, accuracy, clear technical writing, reproducibility, and modular architecture. Created a comprehensive constitution with sections for core principles, technical standards, development workflow, and governance.

## Outcome

- ✅ Impact: Created initial project constitution with 6 core principles and governance structure
- 🧪 Tests: none
- 📁 Files: Updated memory/constitution.md with project-specific principles
- 🔁 Next prompts: Project constitution is established, ready for spec development
- 🧠 Reflection: Constitution properly captures the project's core values and technical requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin spec development for the book project