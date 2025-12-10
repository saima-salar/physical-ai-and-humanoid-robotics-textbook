import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatbotModal from '../components/ChatbotModal';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
      </OriginalLayout>
      <ChatbotModal />
    </>
  );
}