import React from 'react';

const SimpleChatbotButton = () => {
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
        background: '#D5BDAF',
        border: 'none',
        borderRadius: '50%', /* Perfect circle */
        width: '30px',
        height: '30px',
        cursor: 'pointer',
        fontSize: '0.9em', /* Smaller size to fit in circle */
        color: '#333333', /* Dark color for contrast */
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        fontWeight: 'bold'
      }}
    >
      💬
    </button>
  );
};

export default SimpleChatbotButton;