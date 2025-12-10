from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import uuid
import logging
import json
import random

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Physical AI and Humanoid Robotics Mock Chatbot",
    description="A mock chatbot for testing frontend functionality",
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

class ChatRequest(BaseModel):
    message: str
    conversation_id: Optional[str] = None
    selected_text: Optional[str] = ""

class ChatResponse(BaseModel):
    conversation_id: str
    response: str
    sources: Optional[List[dict]] = []

class HealthCheck(BaseModel):
    status: str = "ok"

@app.get("/health", response_model=HealthCheck)
async def health_check():
    """Health check endpoint"""
    return HealthCheck()

@app.post("/chat", response_model=ChatResponse)
async def chat(chat_request: ChatRequest):
    """
    Mock chat endpoint that simulates responses about Physical AI and Humanoid Robotics
    """
    try:
        # Generate a conversation ID if not provided
        conversation_id = chat_request.conversation_id or str(uuid.uuid4())

        # Mock responses based on keywords in the user's message
        user_message = chat_request.message.lower()

        if 'introduction' in user_message or 'what is' in user_message or 'define' in user_message:
            response = "Physical AI refers to the integration of artificial intelligence with physical systems, particularly robots. It encompasses the development of intelligent agents that can perceive, reason, and act in the physical world. This includes humanoid robotics, which focuses on creating robots with human-like form and capabilities."
        elif 'robotics' in user_message or 'robot' in user_message:
            response = "Robotics is an interdisciplinary field that combines engineering and computer science to design, construct, operate, and use robots. In the context of Physical AI, robotics focuses on creating machines that can interact with the physical world through sensors and actuators, demonstrating intelligent behavior."
        elif 'humanoid' in user_message or 'human-like' in user_message:
            response = "Humanoid robots are robots with human-like features and form. They are designed to mimic human appearance and behavior, making them ideal for human-robot interaction. These robots often have limbs, a head, and sometimes facial features similar to humans."
        elif 'sensing' in user_message or 'perception' in user_message:
            response = "Sensing and perception in robotics involve the use of various sensors to gather information about the environment. This includes cameras, LIDAR, IMUs, force sensors, and other devices that allow robots to understand their surroundings and make informed decisions."
        elif 'learning' in user_message or 'machine learning' in user_message:
            response = "Machine learning in Physical AI enables robots to improve their performance through experience. This includes reinforcement learning, supervised learning, and other techniques that allow robots to adapt to new situations and optimize their behavior."
        elif 'chapter' in user_message or 'textbook' in user_message:
            response = "The Physical AI and Humanoid Robotics textbook covers foundational concepts, sensing and perception, locomotion, manipulation, learning, and control systems. It provides comprehensive coverage of how AI and robotics intersect to create intelligent physical systems."
        else:
            # Generic responses about Physical AI and robotics
            generic_responses = [
                "Physical AI and Humanoid Robotics is an exciting field that combines artificial intelligence with physical systems. It focuses on creating intelligent robots that can interact with the real world effectively.",
                "The field of Physical AI explores how artificial intelligence can be embodied in physical systems like robots. This includes understanding how robots can learn, adapt, and interact with their environment.",
                "Humanoid robotics is a specialized area of robotics focused on creating robots with human-like characteristics. These robots are designed to interact naturally with humans and operate in human environments.",
                "Robots in the Physical AI domain integrate sensing, planning, and actuation to perform complex tasks in unstructured environments. This requires sophisticated algorithms for perception, decision-making, and control.",
                "The intersection of AI and robotics creates opportunities for machines that can learn from experience, adapt to new situations, and perform tasks that traditionally required human intelligence and dexterity."
            ]
            response = random.choice(generic_responses)

        # Create mock sources
        mock_sources = [
            {"chapter_title": "Chapter 1 - Introduction to Physical AI", "source_file": "chapter-01-introduction-to-physical-ai.md"},
            {"chapter_title": "Chapter 2 - Foundations of Robotics", "source_file": "chapter-02-foundations-of-robotics-for-physical-ai.md"},
            {"chapter_title": "Chapter 6 - Sensing and Perception", "source_file": "chapter-06-sensing-and-perception.md"}
        ]

        # Randomly select 1-2 sources
        selected_sources = random.sample(mock_sources, k=random.randint(1, 2))

        return ChatResponse(
            conversation_id=conversation_id,
            response=response,
            sources=selected_sources
        )
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("mock_backend:app", host="0.0.0.0", port=8000, reload=True)