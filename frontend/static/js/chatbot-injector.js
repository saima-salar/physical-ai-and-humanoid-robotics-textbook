// Chatbot injector that creates a visible button with modal connected to backend
(function() {
  'use strict';

  // Check for existing chatbot elements before creating new ones
  if (document.getElementById('chatbot-standalone') || document.getElementById('chatbot-modal')) {
    // If elements exist, remove them to prevent duplicates
    const existingButton = document.getElementById('chatbot-standalone');
    const existingModal = document.getElementById('chatbot-modal');

    if (existingButton) {
      existingButton.remove();
    }
    if (existingModal) {
      existingModal.remove();
    }
  }

  // Create the chatbot button element
  const chatbotButton = document.createElement('div');
  chatbotButton.id = 'chatbot-standalone';
  chatbotButton.innerHTML = `
    <button id="chatbot-toggle-btn" style="
      position: fixed !important;
      bottom: 24px !important;
      right: 24px !important;
      width: 60px !important;
      height: 60px !important;
      border-radius: 50% !important;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%) !important;
      color: white !important;
      border: none !important;
      font-size: 24px !important;
      cursor: pointer !important;
      box-shadow: 0 10px 30px rgba(102, 126, 234, 0.4) !important;
      z-index: 2147483647 !important;
      display: flex !important;
      align-items: center !important;
      justify-content: center !important;
      transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1) !important;
      transform: scale(1) !important;
      outline: none !important;
      border: 2px solid rgba(255, 255, 255, 0.2) !important;
      backdrop-filter: blur(10px) !important;
      font-weight: normal !important;
    " aria-label="Ask AI Assistant - Open chatbot">
      🤖
    </button>
  `;

  // Create the chatbot modal element
  const chatbotModal = document.createElement('div');
  chatbotModal.id = 'chatbot-modal';
  chatbotModal.style.display = 'none';
  chatbotModal.innerHTML = `
    <div id="chatbot-container" style="
      position: fixed !important;
      bottom: 94px !important; /* 60px button + 24px spacing + 10px buffer */
      right: 24px !important;
      width: 400px !important;
      height: 500px !important;
      background: linear-gradient(135deg, #ffffff 0%, #f0f4ff 100%) !important;
      border-radius: 20px !important;
      box-shadow: 0 25px 50px rgba(102, 126, 234, 0.3) !important;
      z-index: 2147483646 !important; /* Just below button */
      display: flex !important;
      flex-direction: column !important;
      overflow: hidden !important;
      border: 1px solid rgba(102, 126, 234, 0.2) !important;
      backdrop-filter: blur(20px) !important;
    ">
      <div style="
        padding: 16px 20px !important;
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%) !important;
        color: white !important;
        display: flex !important;
        justify-content: space-between !important;
        align-items: center !important;
      ">
        <h3 style="
          margin: 0 !important;
          font-size: 1.1em !important;
          font-weight: 600 !important;
        ">AI Assistant</h3>
        <button id="close-chatbot-btn" style="
          background: rgba(255, 255, 255, 0.25) !important;
          border: none !important;
          color: white !important;
          width: 32px !important;
          height: 32px !important;
          border-radius: 50% !important;
          cursor: pointer !important;
          font-size: 1.2em !important;
          display: flex !important;
          align-items: center !important;
          justify-content: center !important;
          transition: all 0.3s ease !important;
        ">×</button>
      </div>
      <div id="chat-messages" style="
        flex: 1 !important;
        padding: 20px !important;
        overflow-y: auto !important;
        display: flex !important;
        flex-direction: column !important;
        gap: 16px !important;
      ">
        <div style="
          background: white !important;
          padding: 12px 16px !important;
          border-radius: 18px !important;
          max-width: 80% !important;
          align-self: flex-start !important;
          box-shadow: 0 2px 8px rgba(102, 126, 234, 0.1) !important;
        ">
          Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics textbook. How can I help you today?
        </div>
      </div>
      <div style="
        padding: 16px !important;
        border-top: 1px solid rgba(102, 126, 234, 0.1) !important;
        background: white !important;
      ">
        <div style="
          display: flex !important;
          gap: 8px !important;
        ">
          <input id="chat-input" type="text" placeholder="Ask a question..." style="
            flex: 1 !important;
            padding: 12px 16px !important;
            border: 1px solid rgba(102, 126, 234, 0.3) !important;
            border-radius: 12px !important;
            font-size: 14px !important;
          ">
          <button id="send-message-btn" style="
            padding: 12px 20px !important;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%) !important;
            color: white !important;
            border: none !important;
            border-radius: 12px !important;
            cursor: pointer !important;
            font-weight: 500 !important;
          ">Send</button>
        </div>
      </div>
    </div>
  `;

  // Append the button and modal to the body first
  document.body.appendChild(chatbotButton);
  document.body.appendChild(chatbotModal);

  // Add functionality after elements are added to DOM
  document.addEventListener('DOMContentLoaded', function() {
    const button = document.getElementById('chatbot-toggle-btn');
    const modal = document.getElementById('chatbot-modal');
    const closeBtn = document.getElementById('close-chatbot-btn');
    const sendBtn = document.getElementById('send-message-btn');
    const chatInput = document.getElementById('chat-input');
    const chatMessages = document.getElementById('chat-messages');

    // Add hover effect with JavaScript
    if (button) {
      button.addEventListener('mouseenter', function() {
        this.style.transform = 'scale(1.1) translateY(-2px)';
        this.style.boxShadow = '0 15px 40px rgba(102, 126, 234, 0.6)';
      });

      button.addEventListener('mouseleave', function() {
        this.style.transform = 'scale(1) translateY(0px)';
        this.style.boxShadow = '0 10px 30px rgba(102, 126, 234, 0.4)';
      });

      button.addEventListener('click', function() {
        modal.style.display = modal.style.display === 'none' ? 'block' : 'none';
      });
    }

    if (closeBtn) {
      closeBtn.addEventListener('click', function() {
        modal.style.display = 'none';
      });
    }

    if (sendBtn) {
      sendBtn.addEventListener('click', function() {
        sendMessage();
      });
    }

    if (chatInput) {
      chatInput.addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
          sendMessage();
        }
      });
    }

    function addMessage(text, isUser = false) {
      const messageDiv = document.createElement('div');
      messageDiv.style.cssText = `
        padding: 12px 16px !important;
        border-radius: 18px !important;
        max-width: 80% !important;
        align-self: ${isUser ? 'flex-end' : 'flex-start'} !important;
        box-shadow: 0 2px 8px rgba(102, 126, 234, ${isUser ? 0.2 : 0.1}) !important;
        background: ${isUser ? 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)' : 'white'} !important;
        color: ${isUser ? 'white' : 'initial'} !important;
      `;
      messageDiv.textContent = text;
      chatMessages.appendChild(messageDiv);

      // Scroll to bottom
      chatMessages.scrollTop = chatMessages.scrollHeight;
    }

    async function sendMessage() {
      const message = chatInput.value.trim();
      if (!message) return;

      // Add user message
      addMessage(message, true);
      chatInput.value = '';

      try {
        // Show typing indicator
        const typingIndicator = document.createElement('div');
        typingIndicator.id = 'typing-indicator';
        typingIndicator.style.cssText = `
          padding: 12px 16px !important;
          border-radius: 18px !important;
          max-width: 80% !important;
          align-self: flex-start !important;
          background: white !important;
          color: #666 !important;
          font-style: italic !important;
        `;
        typingIndicator.textContent = 'Thinking...';
        chatMessages.appendChild(typingIndicator);
        chatMessages.scrollTop = chatMessages.scrollHeight;

        // Call the backend API
        const response = await fetch('http://localhost:8001/chat', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            message: message,
            conversation_id: null,
            selected_text: ''
          })
        });

        // Remove typing indicator
        if (document.getElementById('typing-indicator')) {
          document.getElementById('typing-indicator').remove();
        }

        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data = await response.json();

        // Add bot response
        addMessage(data.response, false);

      } catch (error) {
        // Remove typing indicator
        if (document.getElementById('typing-indicator')) {
          document.getElementById('typing-indicator').remove();
        }

        // Show error message
        addMessage("Sorry, I'm having trouble connecting to the AI service. Please try again later.", false);
        console.error('Chat error:', error);
      }
    }
  });

  // Add click outside to close functionality
  document.addEventListener('click', function(event) {
    const modal = document.getElementById('chatbot-modal');
    const button = document.getElementById('chatbot-toggle-btn');

    if (modal && modal.style.display === 'block' &&
        !modal.contains(event.target) &&
        event.target !== button) {
      modal.style.display = 'none';
    }
  });
})();