import React, { useState, useEffect } from 'react';

const UrduTranslationButton = ({ isNavbar = false }) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [originalContent, setOriginalContent] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isClient, setIsClient] = useState(false);

  // Set isClient to true on the client side to prevent SSR issues
  useEffect(() => {
    setIsClient(true);
  }, []);

  const translateToUrdu = async () => {
    setIsLoading(true);
    try {
      // Get the content of the current page/chapter to translate
      // We'll extract the main content from the DOM
      if (typeof document !== 'undefined') {
        const mainContentElement = document.querySelector('main .markdown');

        if (mainContentElement) {
          if (isTranslated) {
            // Toggle back to original
            mainContentElement.innerHTML = originalContent;
            setIsTranslated(false);
            return;
          }

          // Store original content before translation
          const contentToTranslate = mainContentElement.innerHTML;
          setOriginalContent(contentToTranslate);

          // Limit the content size to avoid exceeding API limits
          let textToTranslate = contentToTranslate;
          if (textToTranslate.length > 5000) {
            textToTranslate = textToTranslate.substring(0, 5000);
          }

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
              text: textToTranslate,
              target: 'ur',
              source: 'en'
            })
          });

          if (response.ok) {
            const data = await response.json();
            // Replace the content in the DOM with translated content
            mainContentElement.innerHTML = data.translated_text;
            setTranslatedContent(data.translated_text);
            setIsTranslated(true);
          } else {
            console.error('Translation failed:', await response.text());
            alert('Translation failed. Please try again.');
          }
        } else {
          alert('Content not available for translation.');
        }
      } else {
        alert('Translation is only available in the browser.');
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

  // Don't render anything during SSR, only on the client
  if (!isClient) {
    return <div className="urdu-translation-placeholder" style={{height: '40px', marginBottom: '16px'}}></div>;
  }

  return (
    <div className="urdu-translation-component">
      <button
        onClick={translateToUrdu}
        className={`urdu-translation-button ${isNavbar ? 'navbar-btn' : ''}`}
        disabled={isLoading}
        title={isTranslated ? 'View original content' : 'Translate to Urdu'}
      >
        <span className="translation-icon">T</span>
      </button>
    </div>
  );
};

export default UrduTranslationButton;