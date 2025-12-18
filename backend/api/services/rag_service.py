import aiohttp
import asyncio
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging
from config import *

from models.chat_models import DocumentChunk

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        # Check if OpenRouter is available
        if not OPENROUTER_API_KEY:
            logger.warning("OPENROUTER_API_KEY not set. RAG service will run in mock mode.")
            self.openrouter_available = False
            self.embedding_available = False
        else:
            self.openrouter_available = True
            # For now, assume embedding is available if API key is set
            # We'll test embedding functionality at runtime instead of during initialization
            self.embedding_available = True
            logger.info("OpenRouter is configured, embedding availability will be tested at runtime.")

        # Initialize Qdrant client - make it optional
        if not QDRANT_URL or not QDRANT_API_KEY:
            logger.warning("QDRANT_URL or QDRANT_API_KEY not set. RAG service will run in mock mode.")
            self.qdrant_available = False
            self.qdrant_client = None
        else:
            try:
                self.qdrant_client = QdrantClient(
                    url=QDRANT_URL,
                    api_key=QDRANT_API_KEY,
                    prefer_grpc=False  # Using HTTP for simplicity
                )

                # Test connection and initialize the collection
                self._initialize_collection()
                self.qdrant_available = True
            except Exception as e:
                logger.error(f"Qdrant connection failed: {str(e)}. Running in mock mode.")
                self.qdrant_available = False
                self.qdrant_client = None

    def _initialize_collection(self):
        """Initialize the Qdrant collection if it doesn't exist"""
        if not self.qdrant_client:
            return

        try:
            # Check if collection exists
            self.qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
            logger.info(f"Collection '{QDRANT_COLLECTION_NAME}' already exists")
        except:
            try:
                # Create collection if it doesn't exist
                # For OpenRouter embeddings, the vector size may differ from OpenAI's 1536
                # Using a common size of 1536 which should work with most embedding models
                self.qdrant_client.create_collection(
                    collection_name=QDRANT_COLLECTION_NAME,
                    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
                )
                logger.info(f"Created collection '{QDRANT_COLLECTION_NAME}'")
            except Exception as e:
                logger.error(f"Failed to create Qdrant collection: {str(e)}")

    async def get_relevant_chunks(self, query: str, selected_text: Optional[str] = None) -> List[DocumentChunk]:
        """
        Retrieve relevant document chunks based on the query and optional selected text
        """
        if not self.qdrant_available or not self.openrouter_available:
            # Return mock chunks when services are not available
            logger.info("Using mock chunks due to unavailable services")
            return self._get_mock_chunks(query)

        try:
            # Use selected text as additional context if provided
            search_text = f"{query} {selected_text}" if selected_text else query

            # Get embeddings for the query using OpenRouter
            import sys
            import os
            sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
            from docs_processing.embedding_utils import get_embeddings
            embeddings = await get_embeddings([search_text])
            query_embedding = embeddings[0]

            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=QDRANT_COLLECTION_NAME,
                query_vector=query_embedding,
                limit=5,  # Return top 5 most relevant chunks
                with_payload=True
            )

            chunks = []
            for result in search_results:
                chunk = DocumentChunk(
                    id=result.id,
                    content=result.payload.get('content', ''),
                    metadata=result.payload,
                    score=result.score
                )
                chunks.append(chunk)

            return chunks
        except Exception as e:
            logger.error(f"Error in get_relevant_chunks: {str(e)}. Using mock chunks.")
            # Check if the error is related to embedding/api credits and update availability
            error_str = str(e).lower()
            if "credit" in error_str or "402" in error_str or "insufficient" in error_str:
                self.embedding_available = False
                logger.warning("Embedding functionality disabled due to API credits issue.")
            return self._get_mock_chunks(query)

    def _get_mock_chunks(self, query: str) -> List[DocumentChunk]:
        """Return mock chunks for testing when services are unavailable"""
        # Return mock chunks based on keywords in the query
        mock_content = ""
        if "introduction" in query.lower() or "physical ai" in query.lower():
            mock_content = (
                "Physical AI refers to the integration of artificial intelligence with physical systems, "
                "particularly robots. It encompasses the development of intelligent agents that can perceive, "
                "reason, and act in the physical world. This includes humanoid robotics, which focuses on "
                "creating robots with human-like form and capabilities."
            )
        elif "robotics" in query.lower() or "robot" in query.lower():
            mock_content = (
                "Robotics is an interdisciplinary field that combines engineering and computer science to "
                "design, construct, operate, and use robots. In the context of Physical AI, robotics focuses "
                "on creating machines that can interact with the physical world through sensors and actuators, "
                "demonstrating intelligent behavior."
            )
        elif "humanoid" in query.lower() or "human-like" in query.lower():
            mock_content = (
                "Humanoid robots are robots with human-like features and form. They are designed to mimic "
                "human appearance and behavior, making them ideal for human-robot interaction. These robots "
                "often have limbs, a head, and sometimes facial features similar to humans."
            )
        elif "sensing" in query.lower() or "perception" in query.lower():
            mock_content = (
                "Sensing and perception in robotics involve the use of various sensors to gather information "
                "about the environment. This includes cameras, LIDAR, IMUs, force sensors, and other devices "
                "that allow robots to understand their surroundings and make informed decisions."
            )
        else:
            mock_content = (
                "The Physical AI and Humanoid Robotics textbook covers foundational concepts, "
                "sensing and perception, locomotion, manipulation, learning, and control systems. "
                "It provides comprehensive coverage of how AI and robotics intersect to create intelligent physical systems."
            )

        # Create mock chunks
        mock_chunk = DocumentChunk(
            id="mock-1",
            content=mock_content,
            metadata={
                "source_file": "chapter-01-introduction-to-physical-ai.md",
                "chapter_title": "Introduction to Physical AI",
                "score": 0.8
            },
            score=0.8
        )

        return [mock_chunk]

    async def generate_response(self, query: str, context_chunks: List[DocumentChunk], selected_text: Optional[str] = None) -> str:
        """
        Generate a response using OpenRouter based on the query and context
        """
        if not self.openrouter_available:
            # Return mock response when OpenRouter is not available
            logger.info("Using mock response due to unavailable OpenRouter service")
            return self._generate_mock_response(query, context_chunks, selected_text)

        try:
            # Prepare context from retrieved chunks
            context = "\n\n".join([chunk.content for chunk in context_chunks])

            # Build the system message with context
            system_message = f"""
            You are an expert assistant for the Physical AI and Humanoid Robotics textbook.
            Answer questions based on the following context from the textbook:

            {context}

            If the user has selected specific text, also consider that:
            {selected_text if selected_text else 'No selected text provided.'}

            If the answer is not available in the context, clearly state that you don't have enough information from the textbook to answer the question.
            Always provide helpful and accurate responses based on the textbook content.
            """

            # Prepare messages for OpenRouter
            messages = [
                {"role": "system", "content": system_message},
                {"role": "user", "content": query}
            ]

            # Generate response using OpenRouter
            url = "https://openrouter.ai/api/v1/chat/completions"

            headers = {
                "Authorization": f"Bearer {OPENROUTER_API_KEY}",
                "Content-Type": "application/json"
            }

            payload = {
                "model": OPENROUTER_CHAT_MODEL,
                "messages": messages,
                "temperature": 0.7,
                "max_tokens": 1000
            }

            async with aiohttp.ClientSession() as session:
                async with session.post(url, headers=headers, json=payload) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"OpenRouter API error {response.status}: {error_text}")
                        return self._generate_mock_response(query, context_chunks, selected_text)

                    data = await response.json()
                    return data['choices'][0]['message']['content'].strip()

        except Exception as e:
            logger.error(f"Error in generate_response: {str(e)}. Using mock response.")
            return self._generate_mock_response(query, context_chunks, selected_text)

    def _generate_mock_response(self, query: str, context_chunks: List[DocumentChunk], selected_text: Optional[str] = None) -> str:
        """Generate a mock response based on the query when OpenRouter is not available"""
        query_lower = query.lower()

        if "introduction" in query_lower or "what is" in query_lower or "define" in query_lower:
            return "Physical AI refers to the integration of artificial intelligence with physical systems, particularly robots. It encompasses the development of intelligent agents that can perceive, reason, and act in the physical world. This includes humanoid robotics, which focuses on creating robots with human-like form and capabilities."
        elif "robotics" in query_lower or "robot" in query_lower:
            return "Robotics is an interdisciplinary field that combines engineering and computer science to design, construct, operate, and use robots. In the context of Physical AI, robotics focuses on creating machines that can interact with the physical world through sensors and actuators, demonstrating intelligent behavior."
        elif "humanoid" in query_lower or "human-like" in query_lower:
            return "Humanoid robots are robots with human-like features and form. They are designed to mimic human appearance and behavior, making them ideal for human-robot interaction. These robots often have limbs, a head, and sometimes facial features similar to humans."
        elif "sensing" in query_lower or "perception" in query_lower:
            return "Sensing and perception in robotics involve the use of various sensors to gather information about the environment. This includes cameras, LIDAR, IMUs, force sensors, and other devices that allow robots to understand their surroundings and make informed decisions."
        elif "learning" in query_lower or "machine learning" in query_lower:
            return "Machine learning in Physical AI enables robots to improve their performance through experience. This includes reinforcement learning, supervised learning, and other techniques that allow robots to adapt to new situations and optimize their behavior."
        elif "chapter" in query_lower or "textbook" in query_lower:
            return "The Physical AI and Humanoid Robotics textbook covers foundational concepts, sensing and perception, locomotion, manipulation, learning, and control systems. It provides comprehensive coverage of how AI and robotics intersect to create intelligent physical systems."
        else:
            return f"Based on the textbook content: Physical AI and Humanoid Robotics is an exciting field that combines artificial intelligence with physical systems. It focuses on creating intelligent robots that can interact with the real world effectively. The specific information about '{query}' would be found in the relevant chapters of the textbook."

    def add_document_chunk(self, content: str, metadata: dict):
        """
        Add a document chunk to the vector database
        """
        if not self.qdrant_available or not self.openrouter_available:
            logger.warning("Cannot add document chunk: Qdrant or OpenRouter services not available")
            return

        try:
            # Get embedding for the content using OpenRouter
            import sys
            import os
            sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
            from docs_processing.embedding_utils import get_embeddings
            embeddings = asyncio.run(get_embeddings([content]))
            embedding = embeddings[0]

            # Store in Qdrant
            point = models.PointStruct(
                id=metadata.get('id', str(len(self.qdrant_client.scroll(
                    collection_name=QDRANT_COLLECTION_NAME,
                    limit=1
                )[0]) + 1)),
                vector=embedding,
                payload={
                    "content": content,
                    **metadata
                }
            )

            self.qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=[point]
            )
        except Exception as e:
            logger.error(f"Error adding document chunk: {str(e)}")