import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import AuthProvider from '../components/AuthProvider';
import { PersonalizationProvider } from '../components/PersonalizationProvider';

export default function Layout(props) {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        <OriginalLayout {...props}>
          {props.children}
        </OriginalLayout>
      </PersonalizationProvider>
    </AuthProvider>
  );
}