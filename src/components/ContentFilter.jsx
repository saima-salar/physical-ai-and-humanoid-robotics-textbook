import React from 'react';

const ContentFilter = ({ children, preferenceKey, chapterId, defaultShow = true }) => {
  // In a real implementation, this would check user preferences from context/state
  // For now, we'll use localStorage to simulate user preferences
  const getPreference = () => {
    if (typeof window !== 'undefined') {
      const savedPrefs = localStorage.getItem(`chapterPrefs_${chapterId}`);
      if (savedPrefs) {
        const prefs = JSON.parse(savedPrefs);
        if (prefs[preferenceKey] !== undefined) {
          return prefs[preferenceKey];
        }
      }
    }
    return defaultShow;
  };

  const shouldShow = getPreference();

  return shouldShow ? children : null;
};

export default ContentFilter;