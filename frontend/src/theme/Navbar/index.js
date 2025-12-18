import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Custom Navbar to add additional buttons
export default function Navbar(props) {
  return (
    <>
      <OriginalNavbar {...props} />
      <BrowserOnly>
        {() => {
          // Only show additional buttons on docs pages
          if (typeof window !== 'undefined' && window.location.pathname.includes('/docs/')) {
            // Create buttons dynamically and inject them into the navbar
            setTimeout(() => {
              const navbar = document.querySelector('.navbar');
              if (navbar && !document.querySelector('#additional-docs-buttons')) {
                const buttonsContainer = document.createElement('div');
                buttonsContainer.id = 'additional-docs-buttons';
                buttonsContainer.style.display = 'flex';
                buttonsContainer.style.gap = '8px';
                buttonsContainer.style.alignItems = 'center';
                buttonsContainer.style.marginLeft = 'auto'; // Push to the right

                // Translator button
                const translatorBtn = document.createElement('button');
                translatorBtn.innerHTML = '🌐';
                translatorBtn.title = 'Translate to Urdu';
                translatorBtn.style.padding = '6px 10px';
                translatorBtn.style.border = '1px solid #ddd';
                translatorBtn.style.borderRadius = '4px';
                translatorBtn.style.background = 'white';
                translatorBtn.style.cursor = 'pointer';
                translatorBtn.style.fontSize = '14px';

                // Personalization button
                const personalizationBtn = document.createElement('button');
                personalizationBtn.innerHTML = '🎨';
                personalizationBtn.title = 'Personalize Content';
                personalizationBtn.style.padding = '6px 10px';
                personalizationBtn.style.border = '1px solid #ddd';
                personalizationBtn.style.borderRadius = '4px';
                personalizationBtn.style.background = 'white';
                personalizationBtn.style.cursor = 'pointer';
                personalizationBtn.style.fontSize = '14px';

                buttonsContainer.appendChild(translatorBtn);
                buttonsContainer.appendChild(personalizationBtn);

                // Add to the right side of the navbar
                const rightElement = navbar.querySelector('.navbar__items--right');
                if (rightElement) {
                  rightElement.appendChild(buttonsContainer);
                } else {
                  navbar.appendChild(buttonsContainer);
                }
              }
            }, 100);
          }
        }}
      </BrowserOnly>
    </>
  );
}