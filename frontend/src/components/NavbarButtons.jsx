import React from 'react';
import AuthButton from './AuthButton';
import PersonalizationButton from './PersonalizationButton';
import UrduTranslationButton from './UrduTranslationButton';

const NavbarButtons = ({ chapterId, chapterTitle }) => {
  return (
    <div className="navbar-buttons-container" style={{
      display: 'flex',
      gap: '8px',
      zIndex: 100
    }}>
      <div style={{ display: 'inline-block' }}>
        <UrduTranslationButton isNavbar={true} />
      </div>
      <div style={{ display: 'inline-block' }}>
        <PersonalizationButton chapterId={chapterId} chapterTitle={chapterTitle} />
      </div>
      <div style={{ display: 'inline-block' }}>
        <AuthButton />
      </div>
    </div>
  );
};

export default NavbarButtons;