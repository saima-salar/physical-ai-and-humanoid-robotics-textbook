import openai
from typing import List
from config import OPENAI_API_KEY

if OPENAI_API_KEY:
    openai.api_key = OPENAI_API_KEY

async def get_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Get embeddings for a list of texts using OpenAI
    """
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY environment variable is required")

    from openai import OpenAI
    client = OpenAI(api_key=OPENAI_API_KEY)

    response = client.embeddings.create(
        input=texts,
        model="text-embedding-ada-002"
    )

    return [item.embedding for item in response.data]

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