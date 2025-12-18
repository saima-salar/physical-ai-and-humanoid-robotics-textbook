import React, { useState, useEffect } from 'react';
import Chatbot from '@site/src/components/Chatbot';

export default function ChatbotInjector() {
  const [isOpen, setIsOpen] = useState(false);
  const [hasLoaded, setHasLoaded] = useState(false);

  useEffect(() => {
    // Function to handle chatbot toggle
    const handleToggleChatbot = () => {
      console.log('Chatbot toggle clicked! Setting isOpen to:', !isOpen); // Debug log
      setIsOpen(prev => {
        const newState = !prev;
        console.log('Chatbot state changed to:', newState); // Debug log
        return newState;
      });
    };

    // Listen for the custom event to toggle the chatbot
    window.addEventListener('toggleGlobalChatbot', handleToggleChatbot);

    // Use MutationObserver to watch for changes in the DOM and attach event listener when button appears
    const observer = new MutationObserver((mutationsList) => {
      for (let mutation of mutationsList) {
        if (mutation.type === 'childList') {
          const chatbotToggleBtn = document.getElementById('global-chatbot-toggle');
          if (chatbotToggleBtn) {
            // Remove any existing listener to prevent duplicates
            chatbotToggleBtn.removeEventListener('click', handleToggleChatbot);
            // Add the click event listener
            chatbotToggleBtn.addEventListener('click', handleToggleChatbot);
            console.log('Event listener attached to chatbot toggle button'); // Debug log

            // Add visual feedback for debugging
            chatbotToggleBtn.style.cursor = 'pointer';
            chatbotToggleBtn.title = 'Click to open AI Assistant';
            chatbotToggleBtn.style.outline = '2px solid #007cba'; // Visual indicator
          }
        }
      }
    });

    // Start observing the document body for changes
    observer.observe(document.body, {
      childList: true,
      subtree: true
    });

    // Try to attach immediately in case the button already exists
    const immediateAttach = () => {
      const chatbotToggleBtn = document.getElementById('global-chatbot-toggle');
      if (chatbotToggleBtn) {
        chatbotToggleBtn.removeEventListener('click', handleToggleChatbot);
        chatbotToggleBtn.addEventListener('click', handleToggleChatbot);
        console.log('Immediate attachment: Event listener attached to chatbot toggle button'); // Debug log
        chatbotToggleBtn.style.cursor = 'pointer';
        chatbotToggleBtn.title = 'Click to open AI Assistant';
        chatbotToggleBtn.style.outline = '2px solid #007cba'; // Visual indicator
      } else {
        console.log('Immediate attachment: Chatbot toggle button not found yet'); // Debug log
      }
    };

    immediateAttach();

    // Set up an interval as a backup in case MutationObserver doesn't catch it
    const backupInterval = setInterval(() => {
      immediateAttach();
    }, 1000);

    // Only load the chatbot when modal is opened to avoid SSR issues
    if (isOpen) {
      setHasLoaded(true);
    }

    return () => {
      // Clean up event listeners
      window.removeEventListener('toggleGlobalChatbot', handleToggleChatbot);

      // Remove button listener if button exists
      const chatbotToggleBtn = document.getElementById('global-chatbot-toggle');
      if (chatbotToggleBtn) {
        chatbotToggleBtn.removeEventListener('click', handleToggleChatbot);
        console.log('Event listener removed from chatbot toggle button'); // Debug log
      }

      // Disconnect observer and clear interval
      observer.disconnect();
      clearInterval(backupInterval);
    };
  }, [isOpen]);

  // The button is now in the navbar, so we just render the modal part here
  return (
    <>
      {/* Chatbot modal - positioned separately to avoid conflicts */}
      {isOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '24px',
            right: '24px',
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
}