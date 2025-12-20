import React, { createContext, useContext, useState, useEffect } from 'react';

// Create context
const AuthContext = createContext(undefined);

// Custom auth client to work with our custom auth endpoints
const createCustomAuthClient = (baseURL) => {
  return {
    getSession: async () => {
      try {
        const response = await fetch(`${baseURL}/api/auth/session`, {
          method: 'GET',
          credentials: 'include',
          headers: {
            'Content-Type': 'application/json',
          },
        });

        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        return { session: data.session };
      } catch (error) {
        console.error('Error getting session:', error);
        return { session: null };
      }
    },
    signIn: {
      email: async ({ email, password }) => {
        try {
          const response = await fetch(`${baseURL}/api/auth/sign-in/email`, {
            method: 'POST',
            credentials: 'include',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ email, password }),
          });

          if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.error || 'Sign in failed');
          }

          const data = await response.json();
          return data;
        } catch (error) {
          console.error('Sign in error:', error);
          throw error;
        }
      }
    },
    signUp: {
      email: async ({ email, password }) => {
        try {
          const response = await fetch(`${baseURL}/api/auth/sign-up/email`, {
            method: 'POST',
            credentials: 'include',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({ email, password }),
          });

          if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.error || 'Sign up failed');
          }

          const data = await response.json();
          return data;
        } catch (error) {
          console.error('Sign up error:', error);
          throw error;
        }
      }
    },
    signOut: async () => {
      try {
        const response = await fetch(`${baseURL}/api/auth/sign-out`, {
          method: 'POST',
          credentials: 'include',
          headers: {
            'Content-Type': 'application/json',
          },
        });

        if (!response.ok) {
          throw new Error('Sign out failed');
        }

        return await response.json();
      } catch (error) {
        console.error('Sign out error:', error);
        throw error;
      }
    }
  };
};

// Create auth client with lazy initialization to handle environment properly
let authClient;

const getAuthClient = () => {
  if (!authClient) {
    // Determine auth URL - prioritize window.AUTH_API_URL, then window.BACKEND_API_URL, then default
    let AUTH_URL = 'http://localhost:8003';
    if (typeof window !== 'undefined' && window.AUTH_API_URL) {
      AUTH_URL = window.AUTH_API_URL;
    } else if (typeof window !== 'undefined' && window.BACKEND_API_URL) {
      // If we have BACKEND_API_URL from window, use the same URL for auth (auth is integrated)
      AUTH_URL = window.BACKEND_API_URL;
    } else if (typeof process !== 'undefined' && process.env?.BACKEND_API_URL) {
      AUTH_URL = process.env.BACKEND_API_URL || 'http://localhost:8003';
    }

    authClient = createCustomAuthClient(AUTH_URL);
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
      console.log('Attempting to sign in user with email:', email);
      const result = await getAuthClient().signIn.email({
        email,
        password,
        callbackURL: '/', // Redirect after sign in
      });
      console.log('Sign in result:', result);
      setSession(result.session);
      return result;
    } catch (err) {
      console.error('Sign in error details:', err);
      console.error('Sign in error message:', err.message);
      console.error('Sign in error stack:', err.stack);
      setError(err.message || 'Sign in failed');
      throw err;
    }
  };

  const signUp = async (email, password, userData) => {
    setError(null);
    try {
      console.log('Attempting to sign up user with email:', email);
      // Create the user with Better Auth
      const result = await getAuthClient().signUp.email({
        email,
        password,
        callbackURL: '/welcome', // Redirect after sign up
      });

      console.log('Signup result:', result);
      if (result.session) {
        setSession(result.session);
      }

      return result;
    } catch (err) {
      console.error('Sign up error details:', err);
      console.error('Sign up error message:', err.message);
      console.error('Sign up error stack:', err.stack);
      setError(err.message || 'Sign up failed');
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