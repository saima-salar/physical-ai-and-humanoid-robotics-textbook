from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.sql import func
from datetime import datetime
from typing import List
import logging
from config import *

logger = logging.getLogger(__name__)

# Database setup
if not NEON_DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable is required")

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

class ConversationService:
    def __init__(self):
        pass

    async def save_message(self, conversation_id: str, role: str, content: str):
        """Save a message to the database"""
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
            logger.error(f"Error saving message: {str(e)}")
            raise
        finally:
            db.close()

    async def get_conversation(self, conversation_id: str) -> List[dict]:
        """Get conversation history"""
        db = SessionLocal()
        try:
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
            logger.error(f"Error getting conversation: {str(e)}")
            raise
        finally:
            db.close()