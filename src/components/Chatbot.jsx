import React, { useState, useEffect, useRef } from 'react';
import './Chatbot.css';

const Chatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [conversationId, setConversationId] = useState(null);
  const messagesEndRef = useRef(null);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    // Add user message to chat
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          conversation_id: conversationId,
          selected_text: selectedText,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Failed to get response');
      }

      // Update conversation ID if it's the first message
      if (!conversationId) {
        setConversationId(data.conversation_id);
      }

      const botMessage = {
        role: 'assistant',
        content: data.response,
        sources: data.sources || [],
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        role: 'assistant',
        content: `Sorry, I encountered an error: ${error.message}`,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setConversationId(null);
    setSelectedText('');
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Physical AI & Robotics Assistant</h3>
        <button onClick={clearChat} className="clear-chat-btn">Clear Chat</button>
      </div>

      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your Physical AI and Humanoid Robotics textbook assistant.</p>
            <p>Ask me anything about the content, or select text on the page and ask questions about it!</p>
          </div>
        ) : (
          messages.map((message, index) => (
            <div
              key={index}
              className={`message ${message.role === 'user' ? 'user-message' : 'bot-message'}`}
            >
              <div className="message-content">
                {message.content}
              </div>
              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  <details>
                    <summary>Sources</summary>
                    <ul>
                      {message.sources.slice(0, 3).map((source, idx) => (
                        <li key={idx}>{source.chapter_title || 'Chapter'}</li>
                      ))}
                    </ul>
                  </details>
                </div>
              )}
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {selectedText && (
        <div className="selected-text-preview">
          <strong>Selected text for context:</strong>
          <p>"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
        </div>
      )}

      <div className="chatbot-input-area">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the textbook content..."
          rows="3"
          disabled={isLoading}
        />
        <button
          onClick={sendMessage}
          disabled={!inputValue.trim() || isLoading}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default Chatbot;