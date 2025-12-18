# Personalized Content System

This textbook implements a sophisticated personalization system that adapts content based on user preferences and learning profile. The system includes:

## Features

- **User Authentication**: Simulated authentication system that tracks user preferences
- **Content Filtering**: Show/hide content based on user preferences (code examples, math, practical examples, etc.)
- **Personalized Greetings**: Address users by name throughout the content
- **Domain-Specific Examples**: Tailor content to user's interest area (robotics, AI, etc.)
- **Skill-Level Adaptation**: Adjust complexity based on user's experience level

## Components

### PersonalizationButton
- Toggle panel to customize content preferences
- Saves preferences to localStorage per chapter
- Available in all chapters

### ContentFilter
- Conditionally renders content based on user preferences
- Parameters:
  - `preferenceKey`: Which preference to check (code, math, practical, etc.)
  - `chapterId`: Current chapter identifier
  - `defaultShow`: Whether to show by default if preference isn't set

### PersonalizedGreeting
- Dynamically addresses user by name
- Uses `{name}` placeholder in content to be replaced

### PersonalizedCallout
- Shows colored boxes with personalized content
- Filterable by domain interest

## Usage in Markdown

```md
<PersonalizedGreeting fallback="Learner">Welcome to Physical AI</PersonalizedGreeting>, where we explore how learners can bridge the gap between digital AI and physical systems.

<ContentFilter preferenceKey="code" chapterId="chapter-01-introduction-to-physical-ai" defaultShow={true}>
## Code Example
\`\`\`python
print("Hello, personalized world!")
\`\`\`
</ContentFilter>
```

## Technical Implementation

- User profiles stored in localStorage
- Preferences saved per chapter
- Context provider manages user state
- Components dynamically adapt based on user preferences

## Adding to New Chapters

To enable personalization in a chapter, add these imports at the top:

```md

import ContentFilter from '@site/src/components/ContentFilter';

<PersonalizationButton chapterId="chapter-id" chapterTitle="Chapter Title" />
```

Then use the various personalization components throughout the chapter content.