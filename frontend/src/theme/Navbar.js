import React from 'react';
import Navbar from '@theme-original/Navbar';

// This component wraps the original Navbar
export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props} />
    </>
  );
}