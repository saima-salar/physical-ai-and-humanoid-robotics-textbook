from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import uuid
import logging
import json
from datetime import datetime, timedelta

from config import *
from services.rag_service import RAGService
from services.conversation_service import ConversationService
from models.chat_models import ChatRequest, ChatResponse, Message
from hybrid_translation_service import translation_service

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Physical AI and Humanoid Robotics RAG Chatbot",
    description="A chatbot that answers questions about the Physical AI and Humanoid Robotics textbook using RAG",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your frontend origin
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    allow_origin_regex=r"https?://localhost:\d+"
)

# Initialize services
rag_service = RAGService()
conversation_service = ConversationService()

# User and session models for authentication
class User(BaseModel):
    id: str
    email: str
    name: str
    created_at: Optional[str] = None
    updated_at: Optional[str] = None
    email_verified: Optional[bool] = False

class Session(BaseModel):
    user: User
    expires_at: str
    access_token: str
    refresh_token: Optional[str] = None

class SessionResponse(BaseModel):
    session: Optional[Session] = None
    user: Optional[User] = None

class SignInRequest(BaseModel):
    email: str
    password: str

class SignUpRequest(BaseModel):
    email: str
    password: str
    name: Optional[str] = None

class SignInResponse(BaseModel):
    session: Session
    user: User

class SignUpResponse(BaseModel):
    session: Session
    user: User

class SignOutResponse(BaseModel):
    success: bool

class HealthCheck(BaseModel):
    status: str = "ok"

import os
import json
from pathlib import Path

# File-based storage for mock sessions and users (for development)
# Use absolute path to ensure it works regardless of working directory
STORAGE_FILE = Path(__file__).parent / "auth_storage.json"

def load_storage():
    try:
        if STORAGE_FILE.exists():
            with open(STORAGE_FILE, "r") as f:
                data = json.load(f)
                print(f"Loaded storage from {STORAGE_FILE}: {len(data.get('users', {}))} users, {len(data.get('sessions', {}))} sessions")
                return data.get("users", {}), data.get("sessions", {})
        print(f"Storage file {STORAGE_FILE} does not exist, initializing empty")
    except Exception as e:
        print(f"Error loading storage: {e}")
    return {}, {}

def save_storage(users, sessions):
    try:
        with open(STORAGE_FILE, "w") as f:
            json.dump({"users": users, "sessions": sessions}, f)
        print(f"Saved storage to {STORAGE_FILE}: {len(users)} users, {len(sessions)} sessions")
    except Exception as e:
        print(f"Error saving storage: {e}")

# Load existing data or initialize empty
users_db, active_sessions = load_storage()

@app.get("/health", response_model=HealthCheck)
async def health_check():
    """Health check endpoint"""
    return HealthCheck()

@app.get("/api/auth/session", response_model=SessionResponse)
async def get_session(request: Request):
    """Get current session"""
    # Extract session token from headers or cookies
    auth_header = request.headers.get("authorization")
    if auth_header and auth_header.startswith("Bearer "):
        token = auth_header[7:]
        # Check if this token exists in our sessions
        for session_id, session_data in active_sessions.items():
            if session_data.get("access_token") == token:
                user_data = session_data["user"]
                user = User(
                    id=user_data["id"],
                    email=user_data["email"],
                    name=user_data.get("name", user_data["email"].split('@')[0]),
                    email_verified=user_data.get("email_verified", False),
                    created_at=user_data.get("created_at"),
                    updated_at=user_data.get("updated_at")
                )

                session = Session(
                    user=user,
                    expires_at=session_data["expiresAt"],
                    access_token=token
                )

                return SessionResponse(session=session, user=user)

    # If no valid session found, return null
    return SessionResponse(session=None, user=None)

@app.post("/api/auth/sign-in/email", response_model=SignInResponse)
async def sign_in_email(sign_in_request: SignInRequest):
    """Sign in with email and password"""
    print(f"Sign-in attempt for email: {sign_in_request.email}")
    print(f"Current users_db keys: {list(users_db.keys())}")

    # Direct lookup since we store users by email
    if sign_in_request.email not in users_db:
        print(f"User {sign_in_request.email} not found in users_db, raising 401")
        raise HTTPException(status_code=401, detail="Invalid email or password")

    user_data = users_db[sign_in_request.email]
    print(f"Found user: {user_data}")

    # Verify password (for mock implementation, we'll verify it matches the stored password)
    # In a real implementation, you'd hash and verify the password properly
    print(f"Stored password: {user_data.get('password', 'NOT_FOUND')[:5]}...")
    print(f"Input password: {sign_in_request.password[:5]}...")
    print(f"Password match: {sign_in_request.password == user_data['password']}")

    if sign_in_request.password != user_data["password"]:
        print(f"Password mismatch, raising 401")
        raise HTTPException(status_code=401, detail="Invalid email or password")

    print("Password matched, creating session")

    # Generate tokens
    access_token = f"mock_access_token_{uuid.uuid4()}"

    # Create session
    expires_at = (datetime.utcnow() + timedelta(days=1)).isoformat() + "Z"
    session_id = str(uuid.uuid4())

    session_data = {
        "user": user_data,
        "expiresAt": expires_at,
        "access_token": access_token
    }

    active_sessions[session_id] = session_data
    save_storage(users_db, active_sessions)  # Persist the session data

    user = User(
        id=user_data["id"],
        email=user_data["email"],
        name=user_data.get("name", user_data["email"].split('@')[0]),
        email_verified=user_data.get("email_verified", False),
        created_at=user_data.get("created_at"),
        updated_at=user_data.get("updated_at")
    )

    session = Session(
        user=user,
        expires_at=expires_at,
        access_token=access_token
    )

    return SignInResponse(session=session, user=user)

@app.post("/api/auth/sign-up/email", response_model=SignUpResponse)
async def sign_up_email(sign_up_request: SignUpRequest):
    """Sign up with email and password"""
    # Check if user already exists
    for user_id, user in users_db.items():
        if user["email"] == sign_up_request.email:
            raise HTTPException(status_code=409, detail="User already exists")

    # Create new user
    user_id = str(uuid.uuid4())
    created_at = datetime.utcnow().isoformat() + "Z"

    user_data = {
        "id": user_id,
        "email": sign_up_request.email,
        "name": sign_up_request.name or sign_up_request.email.split('@')[0],
        "password": sign_up_request.password,  # In real app, this should be hashed
        "email_verified": False,
        "created_at": created_at,
        "updated_at": created_at
    }

    users_db[sign_up_request.email] = user_data  # Store by email for easy lookup
    save_storage(users_db, active_sessions)  # Persist the user data

    # Generate tokens
    access_token = f"mock_access_token_{uuid.uuid4()}"

    # Create session
    expires_at = (datetime.utcnow() + timedelta(days=1)).isoformat() + "Z"
    session_id = str(uuid.uuid4())

    session_data = {
        "user": user_data,
        "expiresAt": expires_at,
        "access_token": access_token
    }

    active_sessions[session_id] = session_data
    save_storage(users_db, active_sessions)  # Persist the session data

    user = User(
        id=user_id,
        email=sign_up_request.email,
        name=sign_up_request.name or sign_up_request.email.split('@')[0],
        email_verified=False,
        created_at=created_at,
        updated_at=created_at
    )

    session = Session(
        user=user,
        expires_at=expires_at,
        access_token=access_token
    )

    return SignUpResponse(session=session, user=user)

@app.post("/api/auth/sign-out", response_model=SignOutResponse)
async def sign_out(request: Request):
    """Sign out current user"""
    # Extract session token from headers
    auth_header = request.headers.get("authorization")
    if auth_header and auth_header.startswith("Bearer "):
        token = auth_header[7:]
        # Remove session with this token
        session_to_remove = None
        for session_id, session_data in active_sessions.items():
            if session_data.get("access_token") == token:
                session_to_remove = session_id
                break

        if session_to_remove:
            del active_sessions[session_to_remove]
            save_storage(users_db, active_sessions)  # Persist the changes

    return SignOutResponse(success=True)

# Additional endpoints that Better Auth might expect
@app.post("/api/auth/verify-email")
async def verify_email(request: Request):
    """Verify user email"""
    return {"success": True}

@app.post("/api/auth/forgot-password")
async def forgot_password(request: Request):
    """Handle forgot password request"""
    return {"success": True}

@app.post("/api/auth/reset-password")
async def reset_password(request: Request):
    """Reset user password"""
    return {"success": True}

@app.post("/chat", response_model=ChatResponse)
async def chat(chat_request: ChatRequest):
    """
    Main chat endpoint that processes user queries using RAG
    """
    try:
        # Generate a conversation ID if not provided
        conversation_id = chat_request.conversation_id or str(uuid.uuid4())

        # Get relevant context from the textbook
        context_chunks = await rag_service.get_relevant_chunks(
            chat_request.message,
            selected_text=chat_request.selected_text
        )

        # Generate response using OpenAI
        response = await rag_service.generate_response(
            chat_request.message,
            context_chunks,
            chat_request.selected_text
        )

        # Save the conversation to database
        await conversation_service.save_message(
            conversation_id,
            "user",
            chat_request.message
        )
        await conversation_service.save_message(
            conversation_id,
            "assistant",
            response
        )

        return ChatResponse(
            conversation_id=conversation_id,
            response=response,
            sources=[chunk.metadata for chunk in context_chunks]
        )
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

class TranslationRequest(BaseModel):
    text: str
    target: str = "ur"
    source: str = "en"

class TranslationResponse(BaseModel):
    original_text: str
    translated_text: str
    target_language: str
    source_language: str

@app.post("/translate", response_model=TranslationResponse)
async def translate_text(translation_request: TranslationRequest):
    """
    Translate text to the specified language
    """
    try:
        translated_text = await translation_service.translate_text(
            translation_request.text,
            translation_request.target,
            translation_request.source
        )

        return TranslationResponse(
            original_text=translation_request.text,
            translated_text=translated_text,
            target_language=translation_request.target,
            source_language=translation_request.source
        )
    except Exception as e:
        logger.error(f"Error in translation endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/conversations/{conversation_id}")
async def get_conversation(conversation_id: str):
    """Get conversation history"""
    try:
        messages = await conversation_service.get_conversation(conversation_id)
        return {"conversation_id": conversation_id, "messages": messages}
    except Exception as e:
        logger.error(f"Error getting conversation: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)