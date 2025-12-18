# Authentication Setup with Better Auth

This document describes the authentication system implemented using Better Auth for the Physical AI and Humanoid Robotics textbook project.

## Architecture

The authentication system consists of:

1. **Backend Auth Service**: A separate Node.js service using Better Auth running on port 8002
2. **Frontend Components**: React components for signup, signin, and profile management
3. **Integration**: Seamless integration with the existing personalization system

## Backend Setup

The auth service is located in `backend/auth/` and includes:

- `auth.ts`: Better Auth configuration with custom user fields for personalization
- `src/index.ts`: Main server entry point
- `package.json`: Dependencies for the auth service

### Custom User Fields

The auth service includes additional fields for personalization:

- `softwareExperience`: User's software development background
- `hardwareExperience`: User's hardware/robotics experience
- `programmingLanguages`: Array of programming languages user knows (stored as JSON)
- `roboticsExperience`: User's robotics experience
- `learningGoals`: User's learning goals (stored as JSON)
- `skillLevel`: User's skill level (beginner, intermediate, advanced)
- `domainInterest`: User's domain of interest (robotics, ai, humanoid, etc.)

## Frontend Components

### Core Components

- `AuthProvider.jsx`: Context provider for authentication state
- `SignupForm.jsx`: Form with background questions for new users
- `SigninForm.jsx`: Standard sign-in form
- `ProfilePage.jsx`: User profile management page

### Integration with Personalization

The `PersonalizationProvider.jsx` has been updated to:

- Fetch user profile from auth session
- Use background information for content personalization
- Enhance personalization logic based on user's software/hardware experience

## Pages

- `/signup`: User registration with background questions
- `/signin`: User sign-in
- `/profile`: User profile management

## Environment Variables

- `REACT_APP_AUTH_URL`: Base URL for the auth service (defaults to `http://localhost:8002`)

## Running the Auth Service

To start the auth service:

```bash
cd backend/auth
npm install
npm run dev
```

This will start the auth service on port 8002.

## Personalization Features

The system personalizes content based on:

- User's skill level (beginner, intermediate, advanced)
- Domain of interest (robotics, AI, humanoid, etc.)
- Programming background (adds relevant examples)
- Hardware experience (adds relevant examples)
- User preferences set via the personalization panel

## Security

- Passwords are securely hashed by Better Auth
- Sessions are managed securely
- All authentication requests are properly validated

## Future Enhancements

- Social login providers
- Password reset functionality
- Two-factor authentication
- Admin panel for user management