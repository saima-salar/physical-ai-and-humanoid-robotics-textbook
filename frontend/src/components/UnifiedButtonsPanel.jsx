import React, { useState, useEffect } from 'react';
import PersonalizationButton from './PersonalizationButton';
import UrduTranslationButton from './UrduTranslationButton';
import Chatbot from './Chatbot';

const UnifiedButtonsPanel = ({ chapterId, chapterTitle }) => {
  const [isChatbotOpen, setIsChatbotOpen] = useState(false);
  const [hasLoaded, setHasLoaded] = useState(false);

  useEffect(() => {
    if (isChatbotOpen) {
      setHasLoaded(true);
    }
  }, [isChatbotOpen]);

  return (
    <div className="unified-buttons-panel" style={{
      position: 'fixed',
      top: '70px',  // Below navbar
      right: '20px',
      zIndex: 99,
      display: 'flex',
      alignItems: 'center',
      gap: '8px', // Space between buttons
      padding: '10px',
      backgroundColor: 'rgba(255, 255, 255, 0.9)', // Semi-transparent background
      borderRadius: '12px',
      boxShadow: '0 4px 12px rgba(0,0,0,0.1)',
      backdropFilter: 'blur(10px)'
    }}>
      {/* Personalization Button */}
      <div className="button-wrapper" style={{ display: 'inline-block' }}>
        <PersonalizationButton chapterId={chapterId} chapterTitle={chapterTitle} />
      </div>

      {/* Translation Button */}
      <div className="button-wrapper" style={{ display: 'inline-block' }}>
        <UrduTranslationButton />
      </div>

      {/* Chatbot Button */}
      <div className="button-wrapper" style={{ display: 'inline-block' }}>
        <button
          onClick={() => setIsChatbotOpen(!isChatbotOpen)}
          className="chatbot-toggle-button"
          aria-label="Open chatbot"
        >
          &nbsp;
        </button>
      </div>

      {/* Chatbot Modal - positioned relative to the button panel */}
      {isChatbotOpen && (
        <div
          style={{
            position: 'fixed',
            top: '120px',  // Below the button panel
            right: '20px',
            width: '400px',
            height: '500px',
            background: 'linear-gradient(135deg, #ffffff 0%, #f0f4ff 100%)',
            borderRadius: '20px',
            boxShadow: '0 20px 60px rgba(63, 81, 181, 0.25)',
            zIndex: 1000,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            border: '1px solid rgba(63, 81, 181, 0.15)'
          }}
        >
          <div
            style={{
              padding: '16px 20px',
              background: 'linear-gradient(135deg, #3f51b5 0%, #5c6bc0 100%)',
              borderBottom: '1px solid rgba(255, 255, 255, 0.2)',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{ margin: 0, fontSize: '1.1em', fontWeight: '600', color: 'white', letterSpacing: '0.5px' }}>Physical AI & Robotics</h3>
            <button
              onClick={() => setIsChatbotOpen(false)}
              style={{
                background: 'rgba(255, 255, 255, 0.25)',
                border: 'none',
                fontSize: '1.2em',
                cursor: 'pointer',
                color: 'white',
                width: '32px',
                height: '32px',
                borderRadius: '50%',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                transition: 'all 0.3s ease',
                backdropFilter: 'blur(10px)'
              }}
              onMouseEnter={(e) => {
                e.target.style.background = 'rgba(255, 255, 255, 0.35)';
                e.target.style.transform = 'scale(1.1)';
              }}
              onMouseLeave={(e) => {
                e.target.style.background = 'rgba(255, 255, 255, 0.25)';
                e.target.style.transform = 'scale(1)';
              }}
            >
              ×
            </button>
          </div>

          {hasLoaded && <Chatbot />}
        </div>
      )}

      {/* Overlay when modal is open */}
      {isChatbotOpen && (
        <div
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            backgroundColor: 'rgba(0, 0, 0, 0.1)',
            zIndex: 999,
          }}
          onClick={() => setIsChatbotOpen(false)}
        />
      )}
    </div>
  );
};

export default UnifiedButtonsPanel;