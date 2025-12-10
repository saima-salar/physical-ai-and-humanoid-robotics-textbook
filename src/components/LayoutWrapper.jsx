import React from 'react';
import Layout from '@theme/Layout';
import ChatbotModal from './ChatbotModal';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <ChatbotModal />
    </>
  );
}