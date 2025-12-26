 <!--
Sync Impact Report:
Version: 0.0.0 → 1.0.0
Rationale: Initial constitution creation for AI/Spec-Driven Technical Book with RAG Chatbot project

Modified Principles:
- All principles newly defined (initial creation)

Added Sections:
- Core Principles (7 principles defined)
- Quality Standards
- Technical Constraints
- Governance

Templates Status:
✅ plan-template.md - Constitution Check section aligns
✅ spec-template.md - Requirements structure aligns
✅ tasks-template.md - Task categorization aligns

Follow-up TODOs: None
-->

# AI/Spec-Driven Technical Book with RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development (NON-NEGOTIABLE)

Spec-Kit Plus is the single source of truth for all development activities. Every feature, component, and deliverable MUST originate from a specification document before implementation begins. No code is written without corresponding spec documentation. All changes to requirements MUST be reflected in spec files before implementation.

**Rationale**: Ensures traceability, reproducibility, and accountability. Prevents scope drift and undocumented behavior.

### II. Source-Grounded Facts

All factual claims in the technical book MUST be source-grounded with explicit citations. No assertion, statistic, or technical detail may appear without a verifiable reference. Every citation MUST include source URL, retrieval date, and specific page/section when applicable. The RAG chatbot MUST only retrieve and reference content from the book's indexed material.

**Rationale**: Maintains academic integrity, enables verification, and builds reader trust. Zero hallucination tolerance protects project credibility.

### III. Zero Hallucination Tolerance

The RAG chatbot MUST respond "Not found in the provided text" when the requested information is not present in the indexed book content. The system MUST NOT generate, infer, or extrapolate information beyond the retrieved context. All chatbot responses MUST include references to the specific passages retrieved from the book.

**Rationale**: Ensures users receive only accurate, book-grounded answers. Prevents misinformation and maintains system reliability.

### IV. Deterministic and Reproducible Outputs

All content generation, build processes, and deployments MUST be deterministic and version-controlled. Build steps MUST be documented and executable from a clean checkout. Configuration, prompts, and data processing pipelines MUST be committed to version control. No hidden or undocumented behavior is permitted.

**Rationale**: Enables team collaboration, troubleshooting, and auditing. Ensures consistent results across environments.

### V. Modular and Extensible Architecture

Components MUST be loosely coupled with clear interfaces. The book content, RAG backend, vector database, metadata database, and frontend MUST operate as independent modules with documented contracts. Each component MUST be testable in isolation. Adding new features or swapping components MUST NOT require system-wide refactoring.

**Rationale**: Supports maintainability, scalability, and future enhancements. Reduces risk of cascading failures.

### VI. Production-Grade Code Quality

All project infrastructure code MUST meet production standards: comprehensive error handling, input validation, logging, and documentation. Functions MUST have clear purpose statements and type hints (where applicable). Tests MUST cover happy paths, edge cases, and error conditions. Code MUST pass linting and security scans before deployment.

Educational code examples in documentation may be simplified for clarity and pedagogy, provided they are clearly marked as illustrative examples and are not intended for production use.

**Rationale**: Ensures reliability, security, and long-term maintainability for production systems. Facilitates onboarding and code reviews. Allows educational content to prioritize clarity over production-grade completeness.

### VII. Free-Tier and Open Source Compliance

All infrastructure and dependencies MUST use free or open-source tiers as specified: Qdrant Cloud (Free Tier), Neon Serverless Postgres (Free Tier), GitHub Pages (free hosting), OpenAI API (usage-based). Licensing MUST be verified for all dependencies. No undisclosed paid services or proprietary dependencies permitted.

**Rationale**: Meets project constraints, ensures cost predictability, and maintains transparency.

## Quality Standards

### Content Standards

- **Audience**: Developers and software architects
- **Tone**: Professional and instructional
- **Writing Level**: Flesch-Kincaid grade 10–12
- **Plagiarism**: 0% tolerance - all content must be original or properly attributed
- **Structure**: Clear chapter organization with examples, summaries, and learning objectives
- **Accessibility**: Proper heading hierarchy, alt text for images, semantic HTML

### Testing Requirements

- **Unit Tests**: All backend functions and critical frontend logic
- **Integration Tests**: RAG retrieval pipeline, API endpoints, database operations
- **End-to-End Tests**: User workflows (search, chat, content navigation)
- **Content Validation**: Automated checks for broken links, missing citations, formatting errors
- **Performance Tests**: Query latency, concurrent user handling, vector search accuracy

### Documentation Requirements

- **README.md**: Project overview, setup instructions, deployment guide
- **API Documentation**: All endpoints documented with request/response schemas
- **Architecture Diagram**: Visual representation of system components and data flow
- **Deployment Guide**: Step-by-step GitHub Pages deployment with troubleshooting
- **Development Guide**: Local setup, testing procedures, contribution guidelines

## Technical Constraints

### Technology Stack (NON-NEGOTIABLE)

- **Frontend**: Docusaurus (Markdown/MDX) deployed to GitHub Pages
- **Backend**: FastAPI (Python)
- **LLM Integration**: OpenAI Agents or ChatKit SDKs
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Metadata Database**: Neon Serverless Postgres (Free Tier)
- **Version Control**: Git with GitHub hosting

### Performance Targets

- **Page Load**: < 2 seconds for initial book page load
- **Chat Response**: < 3 seconds for RAG chatbot response (p95)
- **Search**: < 1 second for vector similarity search (p95)
- **Build Time**: < 5 minutes for full Docusaurus build
- **Concurrent Users**: Support 50 concurrent users without degradation

### Security Requirements

- **API Keys**: MUST be stored in environment variables, never committed
- **Input Validation**: All user inputs sanitized to prevent injection attacks
- **Rate Limiting**: API endpoints MUST implement rate limiting
- **HTTPS**: All external communication MUST use HTTPS
- **Secrets Management**: Use .env files locally, GitHub Secrets for CI/CD

## Governance

### Amendment Process

1. Proposed changes MUST be documented in a separate file with rationale
2. Changes affecting core principles (Sections I-VII) require explicit approval
3. All amendments MUST include migration plan if existing work is affected
4. Version number MUST be incremented following semantic versioning

### Versioning Policy

- **MAJOR (X.0.0)**: Breaking changes to core principles, backward-incompatible governance changes
- **MINOR (1.X.0)**: New principles added, expanded requirements, new quality standards
- **PATCH (1.0.X)**: Clarifications, typo fixes, non-semantic refinements

### Compliance Reviews

- All pull requests MUST verify compliance with this constitution
- Complexity or deviations MUST be justified in writing and approved before merging
- Periodic audits (monthly or per-milestone) to ensure ongoing compliance
- Violations MUST be addressed immediately or explicitly documented with remediation plan

### Constitution Enforcement

This constitution supersedes all other practices and preferences. When conflicts arise between this document and other guidance (README, comments, verbal instructions), the constitution takes precedence. Proposed shortcuts or "temporary" deviations MUST be reviewed against these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-22 | **Last Amended**: 2025-12-22
