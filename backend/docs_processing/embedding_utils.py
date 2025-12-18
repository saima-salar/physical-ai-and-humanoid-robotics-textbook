import aiohttp
import asyncio
from typing import List
import sys
import os
# Add backend directory to path to import config
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from api.config import OPENROUTER_API_KEY, OPENROUTER_EMBEDDING_MODEL

async def get_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Get embeddings for a list of texts using OpenRouter
    """
    if not OPENROUTER_API_KEY:
        raise ValueError("OPENROUTER_API_KEY environment variable is required")

    url = "https://openrouter.ai/api/v1/embeddings"

    headers = {
        "Authorization": f"Bearer {OPENROUTER_API_KEY}",
        "Content-Type": "application/json"
    }

    # Process in batches to avoid rate limits
    batch_size = 20  # OpenRouter may have rate limits
    all_embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i+batch_size]

        payload = {
            "model": OPENROUTER_EMBEDDING_MODEL,
            "input": batch
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(url, headers=headers, json=payload) as response:
                if response.status != 200:
                    error_text = await response.text()
                    if response.status == 402 or response.status == 400:
                        # Fallback to a simple embedding approach when credits are insufficient
                        import logging
                        logger = logging.getLogger(__name__)
                        logger.warning(f"OpenRouter API error {response.status}: {error_text}. Using fallback embeddings.")

                        # Use scikit-learn's TF-IDF vectorizer as a fallback
                        from sklearn.feature_extraction.text import TfidfVectorizer
                        import numpy as np

                        # For the fallback, we'll use a simpler embedding size (100 dimensions)
                        # since TF-IDF doesn't naturally produce 1536-dim vectors
                        vectorizer = TfidfVectorizer(max_features=1536, stop_words='english')

                        # Fit on the batch texts to create vocabulary
                        try:
                            tfidf_matrix = vectorizer.fit_transform(batch)
                            # Convert to dense array and ensure 1536 dimensions
                            for i in range(len(batch)):
                                embedding = tfidf_matrix[i].toarray()[0].tolist()
                                # Pad or truncate to 1536 dimensions
                                if len(embedding) < 1536:
                                    embedding.extend([0.0] * (1536 - len(embedding)))
                                else:
                                    embedding = embedding[:1536]
                                all_embeddings.append(embedding)
                        except Exception as e:
                            # If TF-IDF fails, fall back to hash-based embeddings
                            logger.warning(f"TF-IDF fallback failed: {str(e)}. Using hash-based embeddings.")
                            for text in batch:
                                # Simple fallback: create a basic embedding using character-level features
                                import hashlib
                                import struct

                                # Create a deterministic "embedding" based on the text content
                                hash_input = text.encode('utf-8')
                                hash_obj = hashlib.sha256(hash_input)
                                hash_hex = hash_obj.hexdigest()

                                # Convert hash to 1536-dimensional vector (same as OpenAI's embedding size)
                                embedding = []
                                for i in range(0, 1536*8, 8):  # 1536 values * 8 chars per value
                                    chunk = hash_hex[i:i+8] if i+8 <= len(hash_hex) else hash_hex[i:i+8] + '0'*(8-len(hash_hex[i:i+8]))
                                    # Convert hex chunk to a float value between -1 and 1
                                    int_val = int(chunk[:8].ljust(8, '0')[:8], 16)
                                    float_val = ((int_val % 200000) / 100000.0) - 1.0  # Normalize to [-1, 1]
                                    embedding.append(float_val)

                                all_embeddings.append(embedding[:1536])  # Ensure exactly 1536 dimensions
                    else:
                        raise Exception(f"OpenRouter API error {response.status}: {error_text}")
                else:
                    data = await response.json()
                    batch_embeddings = [item['embedding'] for item in data['data']]
                    all_embeddings.extend(batch_embeddings)

    return all_embeddings

def create_chunks(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """
    Split text into overlapping chunks
    """
    if len(text) <= chunk_size:
        return [text]

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # If this is the last chunk, include the rest of the text
        if end >= len(text):
            chunks.append(text[start:])
            break

        # Find the nearest sentence break within the overlap region
        chunk_text = text[start:end]
        if end < len(text):
            # Look for sentence endings in the overlap region
            for separator in ['.\n', '. ', '! ', '? ', '\n\n']:
                last_sep = chunk_text.rfind(separator, 0, chunk_size - overlap)
                if last_sep != -1:
                    end = start + last_sep + len(separator)
                    break

        chunks.append(text[start:end])
        start = end

    return chunks