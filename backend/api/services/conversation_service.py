from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.sql import func
from datetime import datetime
from typing import List
import logging
from config import *

logger = logging.getLogger(__name__)

# Database setup - make it optional
if NEON_DATABASE_URL:
    try:
        engine = create_engine(NEON_DATABASE_URL)
        SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
        Base = declarative_base()

        class Conversation(Base):
            __tablename__ = "conversations"

            id = Column(Integer, primary_key=True, index=True)
            conversation_id = Column(String, index=True)
            role = Column(String)
            content = Column(Text)
            timestamp = Column(DateTime(timezone=True), server_default=func.now())

        # Create tables
        Base.metadata.create_all(bind=engine)
        DB_AVAILABLE = True
    except Exception as e:
        logger.warning(f"Database connection failed: {str(e)}. Running in mock mode.")
        DB_AVAILABLE = False
        SessionLocal = None
        Conversation = None
        engine = None
else:
    logger.info("No database URL provided. Running in mock mode.")
    DB_AVAILABLE = False
    SessionLocal = None
    Conversation = None
    engine = None

class ConversationService:
    def __init__(self):
        # Simple in-memory storage for mock mode
        self._conversations = {}

    async def save_message(self, conversation_id: str, role: str, content: str):
        """Save a message to the database or in-memory storage"""
        if DB_AVAILABLE and SessionLocal:
            # Use database
            db = SessionLocal()
            try:
                conversation = Conversation(
                    conversation_id=conversation_id,
                    role=role,
                    content=content
                )
                db.add(conversation)
                db.commit()
            except Exception as e:
                db.rollback()
                logger.error(f"Error saving message to database: {str(e)}")
                # Fallback to in-memory storage
                await self._save_message_in_memory(conversation_id, role, content)
                raise
            finally:
                db.close()
        else:
            # Use in-memory storage
            await self._save_message_in_memory(conversation_id, role, content)

    async def _save_message_in_memory(self, conversation_id: str, role: str, content: str):
        """Save message to in-memory storage"""
        if conversation_id not in self._conversations:
            self._conversations[conversation_id] = []

        self._conversations[conversation_id].append({
            "role": role,
            "content": content,
            "timestamp": datetime.now().isoformat()
        })

    async def get_conversation(self, conversation_id: str) -> List[dict]:
        """Get conversation history from database or in-memory storage"""
        if DB_AVAILABLE and SessionLocal:
            # Try to get from database
            try:
                db = SessionLocal()
                conversations = db.query(Conversation).filter(
                    Conversation.conversation_id == conversation_id
                ).order_by(Conversation.timestamp).all()

                return [
                    {
                        "role": conv.role,
                        "content": conv.content,
                        "timestamp": conv.timestamp.isoformat()
                    }
                    for conv in conversations
                ]
            except Exception as e:
                logger.error(f"Error getting conversation from database: {str(e)}")
                # Fallback to in-memory storage
                return await self._get_conversation_in_memory(conversation_id)
            finally:
                db.close()
        else:
            # Use in-memory storage
            return await self._get_conversation_in_memory(conversation_id)

    async def _get_conversation_in_memory(self, conversation_id: str) -> List[dict]:
        """Get conversation from in-memory storage"""
        return self._conversations.get(conversation_id, [])