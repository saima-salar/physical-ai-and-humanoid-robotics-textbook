import React from 'react';

// Standalone chatbot button that communicates with the global ChatbotModal
const InlineChatbotButton = () => {
  const toggleChatbot = () => {
    // Dispatch a custom event that the main ChatbotModal can listen to
    window.dispatchEvent(new CustomEvent('toggleGlobalChatbot'));
  };

  return (
    <button
      onClick={toggleChatbot}
      className="chatbot-toggle-button"
      aria-label="Open chatbot"
      style={{
        display: 'inline-block',
        verticalAlign: 'middle'
      }}
    >
      &nbsp;
    </button>
  );
};

export default InlineChatbotButton;