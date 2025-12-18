import React, { useState } from 'react';

const TranslationButton = ({ children, chapterContent }) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const translateToUrdu = async () => {
    if (isTranslated) {
      // Toggle back to original
      setIsTranslated(false);
      return;
    }

    setIsLoading(true);
    try {
      // Use the backend translation endpoint
      const contentToTranslate = chapterContent || (typeof children === 'string' ? children :
        (children.props && children.props.children ? children.props.children : children));

      // Determine backend API URL - prioritize window object, then environment variable, then default
      let BACKEND_API_URL = 'http://localhost:8001';
      if (typeof window !== 'undefined' && window.BACKEND_API_URL) {
        BACKEND_API_URL = window.BACKEND_API_URL;
      } else if (typeof process !== 'undefined' && process.env?.BACKEND_API_URL) {
        BACKEND_API_URL = process.env.BACKEND_API_URL;
      }

      const response = await fetch(`${BACKEND_API_URL}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: contentToTranslate,
          target: 'ur',
          source: 'en'
        })
      });

      if (response.ok) {
        const data = await response.json();
        setTranslatedContent(data.translated_text);
        setIsTranslated(true);
      } else {
        console.error('Translation failed:', await response.text());
        alert('Translation failed. Please try again.');
      }
    } catch (error) {
      console.error('Translation error:', error);
      alert('Translation error occurred. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const buttonText = isLoading
    ? 'ترجمہ ہو رہا ہے...' // "Translating..." in Urdu
    : isTranslated
      ? 'اصل متن دیکھیں' // "View original" in Urdu
      : 'اردو میں ترجمہ کریں'; // "Translate to Urdu" in Urdu

  return (
    <div className="translation-component">
      <button
        onClick={translateToUrdu}
        className="translation-button"
        disabled={isLoading}
        style={{
          backgroundColor: '#4CAF50',
          color: 'white',
          padding: '10px 16px',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer',
          marginBottom: '16px',
          fontSize: '14px',
          fontWeight: '500',
          boxShadow: '0 2px 4px rgba(0,0,0,0.2)',
          transition: 'background-color 0.3s'
        }}
        onMouseOver={(e) => e.target.style.backgroundColor = '#45a049'}
        onMouseOut={(e) => e.target.style.backgroundColor = '#4CAF50'}
      >
        {buttonText}
      </button>

      {isTranslated ? (
        <div className="translated-content" dir="rtl" style={{
          direction: 'rtl',
          textAlign: 'right',
          fontFamily: '"Jameel Noori Nastaleeq", "Nafees Web Naskh", "Urdu Typesetting", Arial, sans-serif',
          lineHeight: '1.8',
          fontSize: '16px',
          backgroundColor: '#f9f9f9',
          padding: '20px',
          borderRadius: '8px',
          border: '1px solid #e0e0e0'
        }}>
          {translatedContent}
        </div>
      ) : (
        <div className="original-content">
          {children}
        </div>
      )}
    </div>
  );
};

export default TranslationButton;