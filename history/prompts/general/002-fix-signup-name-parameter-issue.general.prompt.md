---
id: "002"
title: "Fix Sign Up Name Parameter Issue"
stage: "refactor"
date_iso: "2025-12-20"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "Claude Code"
command: "fix-auth-signup"
labels: ["auth", "signup", "frontend", "api", "parameter"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - frontend/src/components/AuthProvider.jsx
tests_yaml: |
  - "Tested direct API call to sign-up endpoint with name parameter"
  - "Verified name is stored in auth_storage.json"
  - "Confirmed frontend can now pass name parameter correctly"
---

# Prompt History Record: Fix Sign Up Name Parameter Issue

## User Prompt
```
Sign up failed. Please try again.
```

## Response Text
Fixed the sign-up functionality by updating the AuthProvider.jsx to properly pass the name parameter from the frontend to the backend API. The issue was that the custom auth client in AuthProvider.jsx was only sending email and password to the sign-up endpoint, but not the name field. I updated both the custom auth client's signUp.email method to accept and send the name parameter, and updated the AuthProvider's signUp function to properly pass the name from the userData object to the API call.

## Outcome
The sign-up functionality now properly handles the name parameter, and users can successfully sign up with their name being stored in the authentication system.

## Evaluation
Successfully tested the fix by making direct API calls to the sign-up endpoint with name parameters. The authentication system now properly stores user names along with email and password, and the frontend can successfully pass all required information to create new accounts.