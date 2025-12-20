import React, { useState, useEffect, useContext } from 'react';
import './PersonalizationButton.css';
import { PersonalizationContext } from './PersonalizationProvider';

const PersonalizationButton = ({ chapterId, chapterTitle }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [userPreferences, setUserPreferences] = useState({});
  const personalizationContext = useContext(PersonalizationContext);

    // Safely access user profile from the context with fallback
  const userProfile = personalizationContext?.userProfile || { is_authenticated: false };
  const { isPersonalizationEnabled, setIsPersonalizationEnabled, updateChapterPreferences, togglePersonalization: globalTogglePersonalization } = personalizationContext || {};

  // Check if user is authenticated using the context
  const isAuthenticated = userProfile.is_authenticated || false;

  // Load saved preferences for this chapter
  useEffect(() => {
    const savedPrefs = localStorage.getItem(`chapterPrefs_${chapterId}`);
    if (savedPrefs) {
      setUserPreferences(JSON.parse(savedPrefs));
    }
  }, [chapterId]);

  const togglePersonalizationPanel = () => {
    // Prevent any errors from breaking the functionality
    try {
      if (!isAuthenticated) {
        alert('Please log in to personalize content');
        return;
      }
      setIsOpen(!isOpen);
    } catch (error) {
      console.error('Error in togglePersonalizationPanel:', error);
      // Fallback: just toggle the state
      setIsOpen(prev => !prev);
    }
  };

  const handlePreferenceChange = (preferenceKey, value) => {
    const newPreferences = {
      ...userPreferences,
      [preferenceKey]: value
    };
    setUserPreferences(newPreferences);

    // Save to localStorage
    localStorage.setItem(`chapterPrefs_${chapterId}`, JSON.stringify(newPreferences));

    // Update context and enable personalization
    if (setIsPersonalizationEnabled) {
      setIsPersonalizationEnabled(true);
    }

    // Update chapter preferences in context if available
    if (updateChapterPreferences) {
      updateChapterPreferences(chapterId, newPreferences);
    }
  };

  const resetPreferences = () => {
    setUserPreferences({});
    localStorage.removeItem(`chapterPrefs_${chapterId}`);
  };

  if (!isAuthenticated) {
    return (
      <div className="personalization-locked">
        <div className="personalization-locked-content">
          <span className="lock-icon">🔒</span>
          <p>Log in to personalize this content</p>
        </div>
      </div>
    );
  }

  // Toggle the global personalization state
  const toggleGlobalPersonalization = () => {
    if (globalTogglePersonalization) {
      globalTogglePersonalization();
    }
  };

  return (
    <div className="personalization-container">
      <button
        className={`personalize-btn ${isPersonalizationEnabled ? 'enabled' : ''}`}
        onClick={toggleGlobalPersonalization}
        title={isPersonalizationEnabled ? 'Disable Personalization' : 'Enable Personalization'}
      >
        <span className="customize-icon">🔧</span>
        {isPersonalizationEnabled && <span className="indicator-dot on"></span>}
        {!isPersonalizationEnabled && <span className="indicator-dot off"></span>}
      </button>

      {/* Preferences panel - separate from the main toggle */}
      <button
        className={`preferences-btn ${isOpen ? 'active' : ''}`}
        onClick={togglePersonalizationPanel}
        title="Open personalization preferences"
      >
        <span className="preferences-icon">⚙️</span>
      </button>

      {isOpen && (
        <div className="personalization-panel">
          <div className="personalization-header">
            <h3>Customize "{chapterTitle}"</h3>
            <button
              className="close-btn"
              onClick={() => setIsOpen(false)}
              aria-label="Close personalization panel"
            >
              ×
            </button>
          </div>

          <div className="personalization-options">
            <div className="option-group">
              <label className="option-label">
                <input
                  type="checkbox"
                  checked={userPreferences.difficulty !== false}
                  onChange={(e) => handlePreferenceChange('difficulty', e.target.checked)}
                />
                <span className="option-text">Show Advanced Details</span>
              </label>

              <label className="option-label">
                <input
                  type="checkbox"
                  checked={userPreferences.math !== false}
                  onChange={(e) => handlePreferenceChange('math', e.target.checked)}
                />
                <span className="option-text">Include Mathematical Formulas</span>
              </label>

              <label className="option-label">
                <input
                  type="checkbox"
                  checked={userPreferences.practical !== false}
                  onChange={(e) => handlePreferenceChange('practical', e.target.checked)}
                />
                <span className="option-text">Show Practical Examples</span>
              </label>

              <label className="option-label">
                <input
                  type="checkbox"
                  checked={userPreferences.code !== false}
                  onChange={(e) => handlePreferenceChange('code', e.target.checked)}
                />
                <span className="option-text">Include Code Examples</span>
              </label>

              <label className="option-label">
                <input
                  type="checkbox"
                  checked={userPreferences.visual !== false}
                  onChange={(e) => handlePreferenceChange('visual', e.target.checked)}
                />
                <span className="option-text">Show Visual Diagrams</span>
              </label>
            </div>

            <div className="preference-summary">
              <h4>Your Preferences:</h4>
              <ul>
                {userPreferences.difficulty !== false && <li>• Advanced details included</li>}
                {userPreferences.math !== false && <li>• Mathematical formulas included</li>}
                {userPreferences.practical !== false && <li>• Practical examples included</li>}
                {userPreferences.code !== false && <li>• Code examples included</li>}
                {userPreferences.visual !== false && <li>• Visual diagrams included</li>}
                {Object.keys(userPreferences).length === 0 && <li>• Default view</li>}
              </ul>
            </div>

            <div className="reset-section">
              <button
                className="reset-btn"
                onClick={resetPreferences}
              >
                Reset to Default
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizationButton;