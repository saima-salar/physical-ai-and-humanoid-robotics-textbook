import React, { useContext } from 'react';
import { usePersonalization } from './PersonalizationProvider';

// Helper function to get chapter ID from URL
const getChapterIdFromURL = () => {
  if (typeof window !== 'undefined') {
    const pathParts = window.location.pathname.split('/');
    return pathParts[pathParts.length - 1] || pathParts[pathParts.length - 2];
  }
  return '';
};

const getChapterPreferences = (id) => {
  if (!id) return {};
  try {
    const savedPrefs = localStorage.getItem(`chapterPrefs_${id}`);
    return savedPrefs ? JSON.parse(savedPrefs) : {};
  } catch (e) {
    return {};
  }
};

// Personalized greeting component
export const PersonalizedGreeting = ({ children, fallback = "Learner" }) => {
  const { userProfile } = usePersonalization();

  // Replace placeholder with user's name if available
  // Handle server-side rendering by checking if userProfile is available
  const userName = userProfile ? (userProfile.name || fallback) : fallback;
  const personalizedText = children?.toString().replace(/\{name\}/g, userName) ||
                          `${userName}, ${children}`;

  return <span>{personalizedText}</span>;
};

// Skill level filter - shows content based on user's skill level
export const SkillFilter = ({ children, level, skillLevel = 'intermediate' }) => {
  const { userProfile } = usePersonalization();
  const chapterId = getChapterIdFromURL();
  const chapterPrefs = getChapterPreferences(chapterId);

  // Check if userProfile is available (client-side)
  if (!userProfile) {
    // If no userProfile (server-side rendering), show content by default
    return <span>{children}</span>;
  }

  // Determine if content should be shown based on skill level and preferences
  if (skillLevel === 'advanced' && userProfile.skillLevel !== 'advanced') {
    // Show simplified version if user is not advanced
    if (chapterPrefs.difficulty === true) {
      return <span>{children}</span>;
    } else {
      return null;
    }
  }

  if (skillLevel === 'beginner' && userProfile.skillLevel === 'advanced') {
    // Advanced users might want to skip beginner content
    if (chapterPrefs.difficulty === true) {
      return <span>{children}</span>;
    } else {
      return null;
    }
  }

  return <span>{children}</span>;
};

// Domain-specific content
export const DomainContent = ({ children, domain }) => {
  const { userProfile } = usePersonalization();

  // Check if userProfile is available (client-side)
  if (!userProfile) {
    // If no userProfile (server-side rendering), show content by default
    return <span>{children}</span>;
  }

  // Show content only if it matches user's domain interest
  if (userProfile.domain && userProfile.domain !== 'general' && userProfile.domain !== domain) {
    return null;
  }

  return <span>{children}</span>;
};

// Interest-based content
export const InterestContent = ({ children, interest }) => {
  const { userProfile } = usePersonalization();

  // Check if userProfile is available (client-side)
  if (!userProfile) {
    // If no userProfile (server-side rendering), show content by default
    return <span>{children}</span>;
  }

  // Show content if user has this interest
  if (userProfile.interests && !userProfile.interests.includes(interest)) {
    return null;
  }

  return <span>{children}</span>;
};

// Component for personalized examples
export const PersonalizedExample = ({ children, domain, skillLevel }) => {
  const { userProfile } = usePersonalization();
  const chapterPrefs = getChapterPreferences(getChapterIdFromURL());

  // Check if userProfile is available (client-side)
  if (!userProfile) {
    // If no userProfile (server-side rendering), show content by default
    return <div className="personalized-example">{children}</div>;
  }

  // Check if content matches user's domain
  if (domain && userProfile.domain && userProfile.domain !== 'general' && userProfile.domain !== domain) {
    return null;
  }

  // Check skill level filtering
  if (skillLevel) {
    if (skillLevel === 'advanced' && userProfile.skillLevel !== 'advanced' && !chapterPrefs.difficulty) {
      return null;
    }
    if (skillLevel === 'beginner' && userProfile.skillLevel === 'advanced' && !chapterPrefs.difficulty) {
      return null;
    }
  }

  return <div className="personalized-example">{children}</div>;
};

// Component for personalized callouts/note boxes
export const PersonalizedCallout = ({ children, type = "note", domain }) => {
  const { userProfile } = usePersonalization();

  // Replace placeholder with user's name if available
  // Handle server-side rendering by checking if userProfile is available
  let processedChildren = children;
  if (typeof children === 'string') {
    const userName = userProfile && userProfile.name ? userProfile.name : 'Learner';
    processedChildren = children.replace(/\{name\}/g, userName);
  }

  // Don't show if it's for a different domain
  if (domain && userProfile && userProfile.domain && userProfile.domain !== 'general' && userProfile.domain !== domain) {
    return null;
  }

  const calloutStyles = {
    note: { borderLeft: "4px solid #4F46E5", backgroundColor: "#EEF2FF" },
    tip: { borderLeft: "4px solid #10B981", backgroundColor: "#ECFDF5" },
    warning: { borderLeft: "4px solid #F59E0B", backgroundColor: "#FFFBEB" },
    important: { borderLeft: "4px solid #EF4444", backgroundColor: "#FEF2F2" }
  };

  return (
    <div
      style={{
        ...calloutStyles[type],
        padding: "1rem",
        margin: "1rem 0",
        borderRadius: "0 0.375rem 0.375rem 0"
      }}
      className="personalized-callout"
    >
      <strong>{type.charAt(0).toUpperCase() + type.slice(1)}:</strong> {processedChildren}
    </div>
  );
};