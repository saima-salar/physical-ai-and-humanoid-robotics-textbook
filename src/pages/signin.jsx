import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/SigninForm';
import '../components/AuthForm.css';

const SigninPage = () => {
  const handleSigninSuccess = () => {
    // Redirect to the home page or back to where the user came from
    window.location.href = '/';
  };

  return (
    <Layout title="Sign In" description="Sign in to access your personalized learning experience">
      <div className="auth-container">
        <SigninForm onSigninSuccess={handleSigninSuccess} />
      </div>
    </Layout>
  );
};

export default SigninPage;