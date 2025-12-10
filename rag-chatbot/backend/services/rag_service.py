import openai
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging
from config import *

from models.chat_models import DocumentChunk

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        # Initialize OpenAI
        if not OPENAI_API_KEY:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        openai.api_key = OPENAI_API_KEY

        # Initialize Qdrant client
        if not QDRANT_URL:
            raise ValueError("QDRANT_URL environment variable is required")

        self.qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=False  # Using HTTP for simplicity
        )

        # Initialize the collection if it doesn't exist
        self._initialize_collection()

    def _initialize_collection(self):
        """Initialize the Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
            logger.info(f"Collection '{QDRANT_COLLECTION_NAME}' already exists")
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=QDRANT_COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # OpenAI embedding size
            )
            logger.info(f"Created collection '{QDRANT_COLLECTION_NAME}'")

    async def get_relevant_chunks(self, query: str, selected_text: Optional[str] = None) -> List[DocumentChunk]:
        """
        Retrieve relevant document chunks based on the query and optional selected text
        """
        # Use selected text as additional context if provided
        search_text = f"{query} {selected_text}" if selected_text else query

        # Get embeddings for the query
        response = openai.Embedding.create(
            input=search_text,
            model="text-embedding-ada-002"
        )
        query_embedding = response['data'][0]['embedding']

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

    async def generate_response(self, query: str, context_chunks: List[DocumentChunk], selected_text: Optional[str] = None) -> str:
        """
        Generate a response using OpenAI based on the query and context
        """
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

        # Prepare messages for OpenAI
        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": query}
        ]

        # Generate response using OpenAI
        from openai import OpenAI
        client = OpenAI(api_key=OPENAI_API_KEY)

        response = client.chat.completions.create(
            model=OPENAI_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        return response.choices[0].message.content.strip()

    def add_document_chunk(self, content: str, metadata: dict):
        """
        Add a document chunk to the vector database
        """
        # Get embedding for the content
        response = openai.Embedding.create(
            input=content,
            model="text-embedding-ada-002"
        )
        embedding = response['data'][0]['embedding']

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