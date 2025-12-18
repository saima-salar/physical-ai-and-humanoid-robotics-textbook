import React from 'react';
import PersonalizationButton from './PersonalizationButton';
import UrduTranslationButton from './UrduTranslationButton';

const AllButtonsPanel = ({ chapterId, chapterTitle }) => {
  return (
    <div className="all-buttons-panel" style={{
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
      backdropFilter: 'blur(10px)' // For modern glass effect if supported
    }}>
      <div className="button-wrapper" style={{ display: 'inline-block' }}>
        <PersonalizationButton chapterId={chapterId} chapterTitle={chapterTitle} />
      </div>
      <div className="button-wrapper" style={{ display: 'inline-block' }}>
        <UrduTranslationButton />
      </div>
      {/* Placeholder for chatbot button - it will be positioned to align with this container */}
      <div id="chatbot-button-placeholder" className="button-wrapper" style={{ display: 'inline-block' }}></div>
    </div>
  );
};

export default AllButtonsPanel;