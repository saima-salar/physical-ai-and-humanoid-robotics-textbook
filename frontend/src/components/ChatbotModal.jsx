import React, { useState, useEffect } from 'react';
import Chatbot from '@site/src/components/Chatbot';

const ChatbotModal = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [hasLoaded, setHasLoaded] = useState(false);

  useEffect(() => {
    // Only load the chatbot when modal is opened to avoid SSR issues
    if (isOpen) {
      setHasLoaded(true);
    }
  }, [isOpen]);

  return (
    <>
      {/* Chatbot button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="chatbot-toggle-button"
        aria-label="Open chatbot"
      >
        &nbsp;
      </button>

      {/* Chatbot modal */}
      {isOpen && (
        <div
          style={{
            position: 'absolute',  // Changed from fixed to absolute for positioning in navbar area
            top: '60px',           // Position below navbar
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
              onClick={() => setIsOpen(false)}
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
      {isOpen && (
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
          onClick={() => setIsOpen(false)}
        />
      )}
    </>
  );
};

export default ChatbotModal;