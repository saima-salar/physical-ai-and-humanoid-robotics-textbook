import React from 'react';
import PersonalizationButton from './PersonalizationButton';
import UrduTranslationButton from './UrduTranslationButton';

const PostNavbarButtons = ({ chapterId, chapterTitle }) => {
  return (
    <div className="post-navbar-buttons" style={{
      display: 'flex',
      justifyContent: 'flex-end',  // Right-aligned
      alignItems: 'center',
      padding: '8px 20px',
      backgroundColor: '#f8f9fa', // Light background to distinguish from navbar
      borderBottom: '1px solid #e0e0e0',
      gap: '8px' // Space between buttons
    }}>
      <div style={{ display: 'inline-block' }}>
        <UrduTranslationButton isNavbar={true} />
      </div>
      <div style={{ display: 'inline-block' }}>
        <PersonalizationButton chapterId={chapterId} chapterTitle={chapterTitle} />
      </div>
    </div>
  );
};

export default PostNavbarButtons;