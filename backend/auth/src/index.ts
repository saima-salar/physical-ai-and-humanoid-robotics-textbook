// Proper authentication server with credential validation
import { Hono } from "hono";
import { serve } from "@hono/node-server";
import { cors } from "hono/cors";
import { createHash } from "crypto";

// Simple in-memory user storage (in production, use a database)
const users: Record<string, { email: string; passwordHash: string; name?: string }> = {};
const sessions: Record<string, { userId: string; expiresAt: Date }> = {};

const app = new Hono();

// Add CORS middleware
app.use("*", cors({
  origin: ["http://localhost:3000", "http://localhost:3001", "http://localhost:3002"],
  credentials: true,
}));

// Helper function to hash passwords
function hashPassword(password: string): string {
  return createHash('sha256').update(password).digest('hex');
}

// Helper function to generate session ID
function generateSessionId(): string {
  return Math.random().toString(36).substring(2, 15) + Math.random().toString(36).substring(2, 15);
}

// Session endpoint
app.get("/api/auth/session", async (c) => {
  const sessionId = c.req.header('Authorization')?.replace('Bearer ', '');
  if (!sessionId) {
    return c.json({ session: null });
  }

  const session = sessions[sessionId];
  if (!session || session.expiresAt < new Date()) {
    return c.json({ session: null });
  }

  const user = users[session.userId];
  if (!user) {
    return c.json({ session: null });
  }

  return c.json({
    session: {
      user: {
        id: session.userId,
        email: user.email,
        name: user.name || user.email.split('@')[0]
      },
      expiresAt: session.expiresAt.toISOString()
    }
  });
});

// Sign in endpoint with proper credential validation
app.post("/api/auth/sign-in/email", async (c) => {
  const body = await c.req.json();
  const { email, password } = body;

  // Validate input
  if (!email || !password) {
    return c.json({ error: "Email and password are required" }, 400);
  }

  // Find user by email
  const user = Object.values(users).find(u => u.email === email);
  if (!user) {
    return c.json({ error: "Invalid email or password" }, 401);
  }

  // Validate password
  const passwordHash = hashPassword(password);
  if (user.passwordHash !== passwordHash) {
    return c.json({ error: "Invalid email or password" }, 401);
  }

  // Create session
  const sessionId = generateSessionId();
  const expiresAt = new Date();
  expiresAt.setDate(expiresAt.getDate() + 1); // 24 hours

  sessions[sessionId] = {
    userId: Object.keys(users).find(key => users[key] === user)!,
    expiresAt
  };

  return c.json({
    session: {
      user: {
        id: Object.keys(users).find(key => users[key] === user)!,
        email: user.email,
        name: user.name || user.email.split('@')[0]
      },
      expiresAt: expiresAt.toISOString()
    }
  });
});

// Sign up endpoint
app.post("/api/auth/sign-up/email", async (c) => {
  const body = await c.req.json();
  const { email, password, name } = body;

  // Validate input
  if (!email || !password) {
    return c.json({ error: "Email and password are required" }, 400);
  }

  // Check if user already exists
  const existingUser = Object.values(users).find(u => u.email === email);
  if (existingUser) {
    return c.json({ error: "User already exists" }, 409);
  }

  // Validate password strength (basic validation)
  if (password.length < 6) {
    return c.json({ error: "Password must be at least 6 characters long" }, 400);
  }

  // Create new user
  const userId = Date.now().toString() + Math.random().toString(36).substring(2, 9);
  const passwordHash = hashPassword(password);

  users[userId] = {
    email,
    passwordHash,
    name
  };

  // Create session
  const sessionId = generateSessionId();
  const expiresAt = new Date();
  expiresAt.setDate(expiresAt.getDate() + 1); // 24 hours

  sessions[sessionId] = {
    userId,
    expiresAt
  };

  return c.json({
    session: {
      user: {
        id: userId,
        email,
        name: name || email.split('@')[0]
      },
      expiresAt: expiresAt.toISOString()
    }
  });
});

// Sign out endpoint
app.post("/api/auth/sign-out", async (c) => {
  const sessionId = c.req.header('Authorization')?.replace('Bearer ', '');
  if (sessionId) {
    delete sessions[sessionId];
  }
  return c.json({ success: true });
});

const port = 9001;

console.log(`Authentication server running on port ${port}`);

// Start the server
serve({
  fetch: app.fetch,
  port,
});

export default app;