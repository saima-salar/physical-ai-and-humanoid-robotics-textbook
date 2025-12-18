import React, { useContext } from 'react';
import { PersonalizationContext } from './PersonalizationProvider';

const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};

const ContentFilter = ({ children, preferenceKey, chapterId, defaultShow = true }) => {
  const { userProfile } = usePersonalization();

  // Get user preferences for this chapter
  const getChapterPreferences = (id) => {
    if (!id) return {};
    try {
      const savedPrefs = localStorage.getItem(`chapterPrefs_${id}`);
      return savedPrefs ? JSON.parse(savedPrefs) : {};
    } catch (e) {
      return {};
    }
  };

  const chapterPrefs = getChapterPreferences(chapterId);

  // Determine if content should be shown based on preferences
  const shouldShow = chapterPrefs[preferenceKey] ?? defaultShow;

  if (!shouldShow) {
    return null; // Don't render the content
  }

  return <span className="filtered-content">{children}</span>;
};

export default ContentFilter;