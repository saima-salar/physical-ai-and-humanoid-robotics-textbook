import React, { createContext, useContext, useState, useEffect } from 'react';
import { createAuthClient } from 'better-auth/react';

// Create context
const AuthContext = createContext(undefined);

// Create auth client with lazy initialization to handle environment properly
let authClient;

const getAuthClient = () => {
  if (!authClient) {
    // Determine auth URL - prioritize window.AUTH_API_URL, then window.BACKEND_API_URL, then default
    let AUTH_URL = 'http://localhost:8002';
    if (typeof window !== 'undefined' && window.AUTH_API_URL) {
      AUTH_URL = window.AUTH_API_URL;
    } else if (typeof window !== 'undefined' && window.BACKEND_API_URL) {
      // If we have BACKEND_API_URL from window, use a similar port for auth
      AUTH_URL = window.BACKEND_API_URL.replace(/:\d+$/, ':8002');
    } else if (typeof process !== 'undefined' && process.env?.BACKEND_API_URL) {
      AUTH_URL = process.env.BACKEND_API_URL.replace(/:\d+$/, ':8002') || 'http://localhost:8002';
    }

    authClient = createAuthClient({
      baseURL: AUTH_URL,
    });
  }
  return authClient;
};

const AuthProvider = ({ children }) => {
  const [session, setSession] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  // Check session on component mount
  useEffect(() => {
    const checkSession = async () => {
      try {
        const currentSession = await getAuthClient().getSession();
        setSession(currentSession?.session || null);
      } catch (err) {
        console.error('Error checking session:', err);
        setError('Failed to check session');
      } finally {
        setIsLoading(false);
      }
    };

    checkSession();
  }, []);

  const signIn = async (email, password) => {
    setError(null);
    try {
      const result = await getAuthClient().signIn.email({
        email,
        password,
        callbackURL: '/', // Redirect after sign in
      });
      setSession(result.session);
      return result;
    } catch (err) {
      console.error('Sign in error:', err);
      setError('Sign in failed');
      throw err;
    }
  };

  const signUp = async (email, password, userData) => {
    setError(null);
    try {
      // First create the user with Better Auth
      const result = await getAuthClient().signUp.email({
        email,
        password,
        callbackURL: '/welcome', // Redirect after sign up
      });

      // Then update user profile with background information
      if (result.session) {
        await getAuthClient().updateUser({
          ...userData
        });
        setSession(result.session);
      }

      return result;
    } catch (err) {
      console.error('Sign up error:', err);
      setError('Sign up failed');
      throw err;
    }
  };

  const signOut = async () => {
    setError(null);
    try {
      await getAuthClient().signOut();
      setSession(null);
    } catch (err) {
      console.error('Sign out error:', err);
      setError('Sign out failed');
      throw err;
    }
  };

  const value = {
    session,
    signIn,
    signUp,
    signOut,
    isLoading,
    error,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export default AuthProvider;

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};