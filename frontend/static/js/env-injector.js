// Environment variable injector for Docusaurus
// This script runs before other scripts and injects environment variables

// Expose BACKEND_API_URL from Docusaurus config to window object
if (typeof window !== 'undefined') {
  // In the browser context
  window.BACKEND_API_URL = window.location.origin.replace(/:\d+$/, ':8003');

  // For auth service, use the same port as backend API
  window.AUTH_API_URL = window.location.origin.replace(/:\d+$/, ':8003');

  // Define the chatbot modal function directly
  window.openChatbotModal = function() {
    console.log('openChatbotModal function called!');

    // Check if modal already exists
    let modal = document.getElementById('global-chatbot-modal');

    if (modal) {
      // If modal exists, toggle visibility
      if (modal.style.display === 'none' || !modal.style.display) {
        modal.style.display = 'block';
      } else {
        modal.style.display = 'none';
      }
      return;
    }

    // Create the chatbot modal if it doesn't exist
    modal = document.createElement('div');
    modal.id = 'global-chatbot-modal';
    modal.style.cssText = `
      position: fixed !important;
      bottom: 100px !important;
      right: 24px !important;
      width: 400px !important;
      height: 500px !important;
      background: linear-gradient(135deg, #ffffff 0%, #f0f4ff 100%) !important;
      border-radius: 20px !important;
      box-shadow: 0 20px 60px rgba(63, 81, 181, 0.25) !important;
      z-index: 2147483647 !important;
      display: flex !important;
      flex-direction: column !important;
      overflow: hidden !important;
      border: 1px solid rgba(63, 81, 181, 0.15) !important;
      font-family: Inter, system-ui, -apple-system, sans-serif !important;
    `;

    modal.innerHTML = `
      <div style="
        padding: 16px 20px !important;
        background: linear-gradient(135deg, #3f51b5 0%, #5c6bc0 100%) !important;
        color: white !important;
        display: flex !important;
        justify-content: space-between !important;
        align-items: center !important;
      ">
        <h3 style="
          margin: 0 !important;
          font-size: 1.1em !important;
          font-weight: 600 !important;
          color: white !important;
        ">AI Assistant</h3>
        <button onclick="closeChatbotModal()" style="
          background: rgba(255, 255, 255, 0.25) !important;
          border: none !important;
          font-size: 1.2em !important;
          cursor: pointer !important;
          color: white !important;
          width: 32px !important;
          height: 32px !important;
          border-radius: 50% !important;
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
        gap: 12px !important;
        background: white !important;
      ">
        <div class="bot-message" style="
          background: white !important;
          padding: 12px 16px !important;
          border-radius: 18px !important;
          max-width: 80% !important;
          align-self: flex-start !important;
          box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05) !important;
          border: 1px solid rgba(63, 81, 181, 0.1) !important;
          font-size: 14px !important;
        ">
          Hello! I'm your AI assistant for Physical AI and Humanoid Robotics. How can I help you today?
        </div>
      </div>
      <div style="
        padding: 16px !important;
        border-top: 1px solid rgba(63, 81, 181, 0.1) !important;
        background: white !important;
      ">
        <div style="
          display: flex !important;
          gap: 8px !important;
        ">
          <input id="chatbot-input" type="text" placeholder="Ask about robotics, AI, or textbook content..." style="
            flex: 1 !important;
            padding: 12px 16px !important;
            border: 1px solid rgba(63, 81, 181, 0.3) !important;
            border-radius: 12px !important;
            font-size: 14px !important;
            outline: none !important;
          " onkeypress="handleChatInput(event)">
          <button onclick="sendChatMessage()" style="
            padding: 12px 20px !important;
            background: linear-gradient(135deg, #3f51b5 0%, #5c6bc0 100%) !important;
            color: white !important;
            border: none !important;
            border-radius: 12px !important;
            cursor: pointer !important;
            font-weight: 500 !important;
          ">Send</button>
        </div>
      </div>
    `;

    document.body.appendChild(modal);

    // Add click outside to close functionality
    const handleClickOutside = function(event) {
      const modal = document.getElementById('global-chatbot-modal');
      const button = document.getElementById('global-chatbot-toggle');

      if (modal && !modal.contains(event.target) && event.target !== button) {
        closeChatbotModal();
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
  };

  window.closeChatbotModal = function() {
    const modal = document.getElementById('global-chatbot-modal');
    if (modal) {
      modal.style.display = 'none';
    }
  };

  window.handleChatInput = function(event) {
    if (event.key === 'Enter') {
      sendChatMessage();
    }
  };

  window.sendChatMessage = async function() {
    const input = document.getElementById('chatbot-input');
    const messagesContainer = document.getElementById('chat-messages');

    const message = input.value.trim();
    if (!message) return;

    // Add user message
    const userMessage = document.createElement('div');
    userMessage.className = 'user-message';
    userMessage.style.cssText = `
      background: linear-gradient(135deg, #3f51b5 0%, #5c6bc0 100%) !important;
      color: white !important;
      padding: 12px 16px !important;
      border-radius: 18px !important;
      max-width: 80% !important;
      align-self: flex-end !important;
      box-shadow: 0 2px 8px rgba(63, 81, 181, 0.2) !important;
      font-size: 14px !important;
    `;
    userMessage.textContent = message;
    messagesContainer.appendChild(userMessage);

    input.value = '';
    messagesContainer.scrollTop = messagesContainer.scrollHeight;

    try {
      // Show typing indicator
      const typingIndicator = document.createElement('div');
      typingIndicator.id = 'typing-indicator';
      typingIndicator.className = 'bot-message';
      typingIndicator.style.cssText = `
        background: white !important;
        padding: 12px 16px !important;
        border-radius: 18px !important;
        max-width: 80% !important;
        align-self: flex-start !important;
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05) !important;
        border: 1px solid rgba(63, 81, 181, 0.1) !important;
        font-size: 14px !important;
        font-style: italic !important;
        color: #666 !important;
      `;
      typingIndicator.textContent = 'Thinking...';
      messagesContainer.appendChild(typingIndicator);
      messagesContainer.scrollTop = messagesContainer.scrollHeight;

      // Call the backend API
      const response = await fetch(window.BACKEND_API_URL + '/chat', {
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
      const typingEl = document.getElementById('typing-indicator');
      if (typingEl) typingEl.remove();

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response
      const botMessage = document.createElement('div');
      botMessage.className = 'bot-message';
      botMessage.style.cssText = `
        background: white !important;
        padding: 12px 16px !important;
        border-radius: 18px !important;
        max-width: 80% !important;
        align-self: flex-start !important;
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05) !important;
        border: 1px solid rgba(63, 81, 181, 0.1) !important;
        font-size: 14px !important;
      `;
      botMessage.textContent = data.response;
      messagesContainer.appendChild(botMessage);
      messagesContainer.scrollTop = messagesContainer.scrollHeight;

    } catch (error) {
      // Remove typing indicator
      const typingEl = document.getElementById('typing-indicator');
      if (typingEl) typingEl.remove();

      // Show error message
      const errorMessage = document.createElement('div');
      errorMessage.className = 'bot-message';
      errorMessage.style.cssText = `
        background: white !important;
        padding: 12px 16px !important;
        border-radius: 18px !important;
        max-width: 80% !important;
        align-self: flex-start !important;
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05) !important;
        border: 1px solid rgba(63, 81, 181, 0.1) !important;
        font-size: 14px !important;
        color: #e53935 !important;
      `;
      errorMessage.textContent = "Sorry, I'm having trouble connecting to the AI service. Please try again.";
      messagesContainer.appendChild(errorMessage);
      messagesContainer.scrollTop = messagesContainer.scrollHeight;

      console.error('Chat error:', error);
    }
  };
}