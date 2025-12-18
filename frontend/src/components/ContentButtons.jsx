import React from 'react';
import PersonalizationButton from './PersonalizationButton';
import UrduTranslationButton from './UrduTranslationButton';
import InlineChatbotButton from './InlineChatbotButton';

const ContentButtons = ({ chapterId, chapterTitle }) => {
  return (
    <span className="content-buttons" style={{
      display: 'inline-flex',
      alignItems: 'center',
      gap: '8px', // Space between buttons
      verticalAlign: 'middle',
      marginLeft: '10px', // Small gap from heading
      position: 'relative',
      top: '2px' // Fine-tune vertical alignment
    }}>
      {/* Translation Button - first in sequence */}
      <span style={{
        display: 'inline-flex',
        alignItems: 'center',
        justifyContent: 'center',
        verticalAlign: 'middle',
        lineHeight: '1',
        order: 1  // Explicitly set order
      }}>
        <UrduTranslationButton />
      </span>
      {/* Personalization Button - second in sequence */}
      <span style={{
        display: 'inline-flex',
        alignItems: 'center',
        justifyContent: 'center',
        verticalAlign: 'middle',
        lineHeight: '1',
        order: 2  // Explicitly set order
      }}>
        <PersonalizationButton chapterId={chapterId} chapterTitle={chapterTitle} />
      </span>
      {/* Chatbot Button - third in sequence */}
      <span style={{
        display: 'inline-flex',
        alignItems: 'center',
        justifyContent: 'center',
        verticalAlign: 'middle',
        lineHeight: '1',
        order: 3  // Explicitly set order
      }}>
        <InlineChatbotButton />
      </span>
    </span>
  );
};

export default ContentButtons;