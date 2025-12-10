import React, { useState, useEffect } from 'react';
import './PersonalizationButton.css';

const PersonalizationButton = ({ chapterId, chapterTitle }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [userPreferences, setUserPreferences] = useState({});
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  // Check if user is authenticated (in a real app, this would come from your auth system)
  useEffect(() => {
    // Simulate checking authentication status
    const checkAuth = () => {
      // In a real implementation, this would check for auth tokens, etc.
      const authStatus = localStorage.getItem('userAuthenticated') === 'true';
      setIsAuthenticated(authStatus);
    };

    checkAuth();

    // Load saved preferences for this chapter
    const savedPrefs = localStorage.getItem(`chapterPrefs_${chapterId}`);
    if (savedPrefs) {
      setUserPreferences(JSON.parse(savedPrefs));
    }
  }, [chapterId]);

  const togglePersonalization = () => {
    if (!isAuthenticated) {
      alert('Please log in to personalize content');
      return;
    }
    setIsOpen(!isOpen);
  };

  const handlePreferenceChange = (preferenceKey, value) => {
    const newPreferences = {
      ...userPreferences,
      [preferenceKey]: value
    };
    setUserPreferences(newPreferences);

    // Save to localStorage
    localStorage.setItem(`chapterPrefs_${chapterId}`, JSON.stringify(newPreferences));
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

  return (
    <div className="personalization-container">
      <button
        className={`personalize-btn ${isOpen ? 'active' : ''}`}
        onClick={togglePersonalization}
        title="Personalize this chapter"
      >
        <span className="customize-icon">🎨</span>
        {isOpen ? 'Hide Personalization' : 'Personalize Content'}
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