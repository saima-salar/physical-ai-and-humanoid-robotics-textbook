from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime

class Message(BaseModel):
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime = datetime.now()

class ChatRequest(BaseModel):
    message: str
    conversation_id: Optional[str] = None
    selected_text: Optional[str] = None  # User-selected text for context
    temperature: float = 0.7

class ChatResponse(BaseModel):
    conversation_id: str
    response: str
    sources: List[Dict[str, Any]] = []

class DocumentChunk(BaseModel):
    id: str
    content: str
    metadata: Dict[str, Any]
    score: Optional[float] = None