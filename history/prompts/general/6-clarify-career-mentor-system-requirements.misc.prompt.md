---
id: 6
title: Clarify Career Mentor System Requirements
stage: misc
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: architext
command: /sp.clarify
labels: ["clarification", "requirements", "career-mentor"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - null
tests:
  - N/A
---

## Prompt

```text
/sp.clarify

You are the Clarification Agent for my project.
The project is a multi-agent "Career Mentor System" built using the OpenAI Agent SDK + Runner.
It includes three main agents:
1) CareerAgent – guides students in career exploration
2) SkillAgent – recommends skills & learning paths
3) JobAgent – provides job guidance
A shared tool: get_career_roadmap()

Your job:
- Identify missing information, gaps, contradictions, or assumptions in the project plan.
- Ask clarifying questions that help complete architecture, workflows, task definitions, dependencies, and chapter alignment.
- Only ask for information that is logically required for building a production-ready system.
- Keep questions structured, grouped, and focused.

Provide:
1) A list of clarification categories (e.g., System Architecture, API details, Agent Behaviors, Tools, Dataset needs, UI/UX, Constraints, Deliverables).
2) 3–5 precise clarification questions per category.
3) Do NOT propose solutions. Only ask
```

## Response snapshot

Here are the clarification categories and questions for your "Career Mentor System" project:

**1. System Architecture**
*   How do the CareerAgent, SkillAgent, and JobAgent communicate and coordinate their actions within the OpenAI Agent SDK + Runner framework?
*   Is there a defined workflow or sequence for agent interaction when processing a multi-faceted user query?
*   What is the error handling and recovery strategy for inter-agent communication failures or tool execution errors?
*   How will the system manage the session state and context across multiple turns of conversation with a user, especially when delegating between agents?

**2. Agent Behaviors & Responsibilities**
*   **CareerAgent**: What specific methodologies or criteria does the CareerAgent use to interpret a user's career aspirations and generate a personalized roadmap?
*   **SkillAgent**: How does the SkillAgent prioritize skill recommendations, and does it consider factors like learning time, market demand, or prerequisite skills?
*   **JobAgent**: What parameters does the JobAgent use to filter and rank job opportunities, and how does it handle discrepancies between skill sets and job requirements?
*   What are the precise boundaries of responsibility for each agent, particularly when a user's query might overlap multiple domains (e.g., "What skills do I need for this job, and how do I get a roadmap for it?")?

**3. API/Tool Details**
*   **get_career_roadmap()**: What are the specific input parameters required by the `get_career_roadmap()` tool, and what is the exact structure of the data it returns?
*   Are there any other tools (internal or external APIs) that the individual agents will leverage, and if so, what are their functionalities, inputs, and outputs?
*   What is the expected latency and throughput of the `get_career_roadmap()` tool, and are there any considerations for scaling its usage?
*   What authentication and authorization mechanisms are in place for `get_career_roadmap()` and any other integrated APIs?

**4. Data Needs**
*   What specific datasets are required for the CareerAgent to generate roadmaps and provide career advice?
*   Which data sources will the SkillAgent use to identify and describe relevant skills, including potential learning resources?
*   What data feeds or APIs will the JobAgent consume to access real-time job market information, including job descriptions, company details, and application links?
*   How will the system handle data privacy and compliance requirements for any personal information provided by the user or gathered during interactions?

**5. Constraints & Non-Functional Requirements**
*   What are the specific performance targets for overall system response time from initial user query to final output?
*   What are the expected user concurrency levels, and how will the system ensure scalability to handle peak loads?
*   What are the linguistic and regional scope requirements for the career mentor system (e.g., English only, global job markets)?
*   Are there any specific platform or technology stack constraints beyond the OpenAI Agent SDK + Runner that must be adhered to?

## Outcome

- ✅ Impact: Clarification questions for the `Career Mentor System` project have been generated based on the user's request.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: The user can now provide answers to these clarification questions.
- 🧠 Reflection: Successfully identified key areas requiring clarification for a multi-agent system project.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): Manual review of clarification questions (PASS).
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A