---
id: 1
title: Updated Humanoid Robotics Textbook Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

Project Name: Humanoid Robotic Textbook — AI-Native Edition

Purpose: Create a clean, concise, professional AI-Native textbook on Humanoid Robotics using Docusaurus, Gemeni API, Spec-Kit Plus, and Claude Code. The book must serve as a fast, simple, high-quality learning resource suitable for students, beginners, and robotics enthusiasts. It must support modern UI features and be compatible with future AI-enhanced learning experiences.

Scope:

The book will contain 12 short, focused chapters covering:
1. Introduction to Humanoid Robotics
2. Foundations of Robotics
3. Humanoid Mechanical Architecture
4. Electronics & Hardware Systems
5. Sensors & Perception
6. Computer Vision Basics
7. Motion Planning & Control
8. Speech & Language Interaction
9. Robotics Programming (Python, C++, ROS2 basics)
10. Practical Humanoid Mini-Project
11. Ethics & Safety in Robotics
12. Future of Humanoid Robots

Core scope requirements:
- Content must stay concise, accurate, and student-friendly.
- Chapters must be small, modular, and easy to index for RAG systems.
- UI must be clean, modern, minimal, and fast.
- Gemini API may be used for diagrams, examples, and concept expansions.
- Entire project must follow Spec-Kit Plus principles (spec-first writing).

Core Principles:

1. Simplicity Over Complexity
   Use simple language, minimal dependencies, clean structure, and short chapters.

2. Content Accuracy & Minimalism (NON-NEGOTIABLE)
   Technical correctness must be verified; avoid over-complication, long equations, or unnecessary theory.

3. Free-Tier-Friendly Architecture
   Book design must remain lightweight and optimized for GitHub Pages deployment and low-compute AI usage.

4. RAG-Ready Textbook
   All content must be written such that a future RAG chatbot can answer ONLY from this textbook without hallucination.

5. Consistent Style & Formatting
   Use Docusaurus Markdown, consistent heading structure, bullet lists, tables, diagrams, and glossary terms.

6. Fast Build & Clean Structure
   Build must complete under 30 seconds; static assets must be minimal; avoid heavy images.

7. Production-Safe Code & Documentation
   Use validated examples, no hardcoded secrets, safe robotics instructions, and correct code snippets.

Key Features:

- Docusaurus-based humanoid robotics textbook with clean, minimal UI.
- Support for:
  - “Select text → Ask AI” interaction (future enhancement)
  - Translation-ready chapters (e.g., Urdu)
  - Personalized content adaptation (optional)
- Gemini-assisted diagrams and explanations.
- Claude Code + Spec-Kit Plus for spec-first chapter development.

Constraints:

- Minimal compute usage.
- Lightweight images/diagrams; avoid large assets.
- Small, clean chapters (beginner-friendly).
- Avoid deep mathematics or unsafe robotics instructions.

Success Criteria:

- Textbook builds successfully with no errors.
- All chapters align with course outcomes and humanoid robotics fundamentals.
- Writing style is consistent across all chapters.
- Claude Code can reliably generate and extend chapters from the specification.
- Deployment to GitHub Pages works smoothly.
- Content is RAG-friendly, clean, and technically correct.

Generate the complete Constitution in clear Markdown format.

## Response snapshot

The project constitution has been successfully updated in `.specify/memory/constitution.md` with version 0.1.0.

## Outcome

- ✅ Impact: Constitution updated successfully.
- 🧪 Tests: No specific tests run for constitution update.
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: Update dependent templates (plan, spec, tasks, commands).
- 🧠 Reflection: PHR created using agent-native tools due to script not found.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` script not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure `create-phr.sh` is available or agent-native fallback is robust.