---
id: "003"
title: "Fix Auth API URL Port Configuration"
stage: "refactor"
date_iso: "2025-12-20"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "Claude Code"
command: "fix-auth-port-config"
labels: ["auth", "configuration", "env", "port", "api"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - frontend/static/js/env-injector.js
tests_yaml: |
  - "Verified AUTH_API_URL now points to correct port (8003)"
  - "Confirmed authentication endpoints are accessible"
  - "Tested sign-up functionality end-to-end"
---

# Prompt History Record: Fix Auth API URL Port Configuration

## User Prompt
```
Sign up failed. Please try again.
```

## Response Text
Identified and fixed the root cause of the authentication failure: the env-injector.js file was setting AUTH_API_URL to port 9001 instead of the correct port 8003 where the backend API is running. The AuthProvider was trying to connect to a non-existent port for authentication endpoints, causing all auth operations to fail. Updated the AUTH_API_URL to use the same port (8003) as BACKEND_API_URL.

## Outcome
Authentication now works correctly as both BACKEND_API_URL and AUTH_API_URL point to the same backend server running on port 8003.

## Evaluation
The authentication system is now properly configured with the correct API endpoints, resolving the "Sign up failed" error that was occurring due to the port mismatch.