---
id: 001
title: Create AI Technical Book Constitution
stage: constitution
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: razam
command: /sp.constitution
labels: ["constitution", "initial-setup", "governance", "spec-driven-development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

```text
/sp.constitution

Project: AI/Spec-Driven Technical Book with RAG Chatbot

Objective:
Create and deploy a spec-driven technical book using Spec-Kit Plus and
Claude Code, published via Docusaurus on GitHub Pages, with an embedded
RAG chatbot that answers questions strictly from the book's content.

Core Principles:
- Spec-first development (Spec-Kit Plus is the single source of truth)
- All factual claims must be source-grounded
- Deterministic, reproducible, version-controlled outputs
- Zero hallucination tolerance in content or chatbot responses
- Modular, extensible architecture

Deliverables:
1) Technical Book
- Authored via Claude Code
- Format: Docusaurus (Markdown/MDX)
- Audience: developers and software architects
- Tone: professional and instructional
- Structure: chapters with examples and summaries
- Writing level: Flesch-Kincaid grade 10–12
- Citations required for all factual claims
- 0% plagiarism tolerance
- Deployed to GitHub Pages with documented build steps

2) RAG Chatbot
- Embedded within the Docusaurus site
- Backend: FastAPI
- LLM: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- Metadata DB: Neon Serverless Postgres
- Retrieval-only answers using book content
- Support "answer only from selected text"
- Respond "Not found in the provided text" when context is missing
- Reference retrieved passages in answers

Constraints:
- Use free/open tiers only where specified
- Production-grade, documented code
- No external or undisclosed datasets
- No undocumented behavior or hidden prompts

Success Criteria:
- Book is live on GitHub Pages
- Content fully traceable to specs and sources
- Chatbot retrieves correctly and never hallucinates
- Repository is clean, reproducible, and review-ready
```

## Response snapshot

Created initial project constitution (v1.0.0) with 7 core principles:

1. **Spec-First Development (NON-NEGOTIABLE)**: Spec-Kit Plus as single source of truth, no code without specs
2. **Source-Grounded Facts**: All claims must have explicit citations, no unsourced content
3. **Zero Hallucination Tolerance**: RAG chatbot responds only from indexed book content
4. **Deterministic and Reproducible Outputs**: All builds and processes must be version-controlled
5. **Modular and Extensible Architecture**: Loosely coupled components with clear interfaces
6. **Production-Grade Code Quality**: Comprehensive error handling, validation, testing, documentation
7. **Free-Tier and Open Source Compliance**: Qdrant Cloud, Neon Postgres, GitHub Pages (free tiers only)

Additional sections:
- **Quality Standards**: Content standards, testing requirements, documentation requirements
- **Technical Constraints**: Technology stack (FastAPI, Docusaurus, OpenAI), performance targets, security requirements
- **Governance**: Amendment process, versioning policy, compliance reviews, enforcement rules

Constitution verified against existing templates (plan-template.md, spec-template.md, tasks-template.md) - all aligned.

File created: `.specify/memory/constitution.md`

## Outcome

- ✅ Impact: Project governance established, all development must now follow constitutional principles
- 🧪 Tests: N/A (governance document)
- 📁 Files: Created `.specify/memory/constitution.md` (v1.0.0)
- 🔁 Next prompts: `/sp.specify` to create first feature specification, or begin book content planning
- 🧠 Reflection: Constitution provides clear guardrails for AI-driven technical book with RAG chatbot. Zero hallucination tolerance and source-grounding principles are critical for maintaining credibility. Free-tier constraints acknowledged and documented.

## Evaluation notes (flywheel)

- Failure modes observed: None (initial creation)
- Graders run and results (PASS/FAIL): N/A (no automated grading for constitution)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test constitution compliance during first feature specification creation
