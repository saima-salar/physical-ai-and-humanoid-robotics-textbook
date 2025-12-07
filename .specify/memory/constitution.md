<!--
Sync Impact Report:
Version change: 0.0.0 → 0.1.0
Modified principles:
- [PRINCIPLE_1_NAME] → Simplicity Over Complexity
- [PRINCIPLE_2_NAME] → Content Accuracy & Minimalism
- [PRINCIPLE_3_NAME] → Free-Tier-Friendly Architecture
- [PRINCIPLE_4_NAME] → RAG-Ready Textbook
- [PRINCIPLE_5_NAME] → Consistent Style & Formatting
- [PRINCIPLE_6_NAME] → Fast Build & Clean Structure
- [PRINCIPLE__DESCRIPTION] → Production-Safe Code & Documentation
Added sections:
- Key Features
- Constraints
- Success Criteria
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Humanoid Robotic Textbook — AI-Native Edition Constitution

## Core Principles

### I. Simplicity Over Complexity
Use simple language, minimal dependencies, clean structure, and short chapters.

### II. Content Accuracy & Minimalism (NON-NEGOTIABLE)
Technical correctness must be verified; avoid over-complication, long equations, or unnecessary theory.

### III. Free-Tier-Friendly Architecture
Book design must remain lightweight and optimized for GitHub Pages deployment and low-compute AI usage.

### IV. RAG-Ready Textbook
All content must be written such that a future RAG chatbot can answer ONLY from this textbook without hallucination.

### V. Consistent Style & Formatting
Use Docusaurus Markdown, consistent heading structure, bullet lists, tables, diagrams, and glossary terms.

### VI. Fast Build & Clean Structure
Build must complete under 30 seconds; static assets must be minimal; avoid heavy images.

### VII. Production-Safe Code & Documentation
Use validated examples, no hardcoded secrets, safe robotics instructions, and correct code snippets.

## Key Features

- Docusaurus-based humanoid robotics textbook with clean, minimal UI.
- Support for:
  - “Select text → Ask AI” interaction (future enhancement)
  - Translation-ready chapters (e.g., Urdu)
  - Personalized content adaptation (optional)
- Gemini-assisted diagrams and explanations.
- Claude Code + Spec-Kit Plus for spec-first chapter development.

## Constraints

- Minimal compute usage.
- Lightweight images/diagrams; avoid large assets.
- Small, clean chapters (beginner-friendly).
- Avoid deep mathematics or unsafe robotics instructions.

## Success Criteria

- Textbook builds successfully with no errors.
- All chapters align with course outcomes and humanoid robotics fundamentals.
- Writing style is consistent across all chapters.
- Claude Code can reliably generate and extend chapters from the specification.
- Deployment to GitHub Pages works smoothly.
- Content is RAG-friendly, clean, and technically correct.

## Governance
Constitution supersedes all other practices; Amendments require documentation, approval, migration plan. All PRs/reviews must verify compliance; Complexity must be justified.

**Version**: 0.1.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05