import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/SignupForm';
import '../components/AuthForm.css';

const SignupPage = () => {
  const handleSignupSuccess = () => {
    // Redirect to a welcome page or back to where the user came from
    window.location.href = '/';
  };

  return (
    <Layout title="Sign Up" description="Create your account to personalize your learning experience">
      <div className="auth-container">
        <SignupForm onSignupSuccess={handleSignupSuccess} />
      </div>
    </Layout>
  );
};

export default SignupPage;