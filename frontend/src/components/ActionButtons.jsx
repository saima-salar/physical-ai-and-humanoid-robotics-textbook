import React from 'react';
import PersonalizationButton from './PersonalizationButton';
import UrduTranslationButton from './UrduTranslationButton';

const ActionButtons = ({ chapterId, chapterTitle }) => {
  return (
    <div className="action-buttons-container" style={{
      position: 'fixed',
      top: '70px',  // Below navbar
      right: '20px',
      zIndex: 99,
      display: 'flex',
      alignItems: 'center',
      gap: '8px', // Reduced space between buttons for compact look
      padding: '10px',
      backgroundColor: 'white', // Or match your theme background
      borderRadius: '12px',
      boxShadow: '0 4px 12px rgba(0,0,0,0.1)',
    }}>
      <PersonalizationButton chapterId={chapterId} chapterTitle={chapterTitle} />
      <UrduTranslationButton />
    </div>
  );
};

export default ActionButtons;