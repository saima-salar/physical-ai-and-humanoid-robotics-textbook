from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import uuid
import logging

from config import *
from services.rag_service import RAGService
from services.conversation_service import ConversationService
from models.chat_models import ChatRequest, ChatResponse, Message

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
)

# Initialize services
rag_service = RAGService()
conversation_service = ConversationService()

class HealthCheck(BaseModel):
    status: str = "ok"

@app.get("/health", response_model=HealthCheck)
async def health_check():
    """Health check endpoint"""
    return HealthCheck()

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