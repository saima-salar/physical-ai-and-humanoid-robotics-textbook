import React from 'react';
import { useAuth } from './AuthProvider';

const AuthButton = () => {
  const { session, isLoading } = useAuth();

  if (isLoading) {
    return (
      <button
        className="auth-button navbar-btn loading"
        disabled
        style={{
          padding: '8px 16px',
          border: '1px solid #ddd',
          borderRadius: '4px',
          backgroundColor: '#f5f5f5',
          color: '#666',
          cursor: 'not-allowed',
          fontSize: '14px',
          fontWeight: '500',
        }}
      >
        Loading...
      </button>
    );
  }

  // Always show login/signup buttons for visibility (simplified approach)
  return (
    <div className="auth-buttons" style={{
      display: 'flex',
      gap: '8px',
      alignItems: 'center'
    }}>
      <a
        href="/signin"
        className="auth-button navbar-btn login"
        style={{
          padding: '8px 16px',
          border: '1px solid #2196F3',
          borderRadius: '20px',
          backgroundColor: 'transparent',
          color: '#2196F3',
          fontSize: '14px',
          fontWeight: '500',
          cursor: 'pointer',
          textDecoration: 'none',
          transition: 'all 0.2s ease'
        }}
        onMouseOver={(e) => {
          e.target.style.backgroundColor = '#2196F3';
          e.target.style.color = 'white';
        }}
        onMouseOut={(e) => {
          e.target.style.backgroundColor = 'transparent';
          e.target.style.color = '#2196F3';
        }}
      >
        Log In
      </a>
      <a
        href="/signup"
        className="auth-button navbar-btn signup"
        style={{
          padding: '8px 16px',
          border: '1px solid #4CAF50',
          borderRadius: '20px',
          backgroundColor: '#4CAF50',
          color: 'white',
          fontSize: '14px',
          fontWeight: '500',
          cursor: 'pointer',
          textDecoration: 'none',
          transition: 'all 0.2s ease'
        }}
        onMouseOver={(e) => {
          e.target.style.backgroundColor = '#45a049';
          e.target.style.borderColor = '#45a049';
        }}
        onMouseOut={(e) => {
          e.target.style.backgroundColor = '#4CAF50';
          e.target.style.borderColor = '#4CAF50';
        }}
      >
        Sign Up
      </a>
    </div>
  );
};

export default AuthButton;