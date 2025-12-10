import os
import asyncio
from pathlib import Path
from typing import List, Dict
import markdown
from bs4 import BeautifulSoup
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models

from config import *
from embedding_utils import create_chunks, get_embeddings

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ChapterProcessor:
    def __init__(self):
        # Initialize Qdrant client
        if not QDRANT_URL:
            raise ValueError("QDRANT_URL environment variable is required")

        self.qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=False
        )

        # Ensure collection exists
        self._ensure_collection()

    def _ensure_collection(self):
        """Ensure the Qdrant collection exists"""
        try:
            self.qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
            logger.info(f"Collection '{QDRANT_COLLECTION_NAME}' already exists")
        except:
            # Create collection
            self.qdrant_client.create_collection(
                collection_name=QDRANT_COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
            )
            logger.info(f"Created collection '{QDRANT_COLLECTION_NAME}'")

    def _extract_text_from_markdown(self, md_content: str) -> str:
        """Extract plain text from markdown content"""
        # Convert markdown to HTML
        html = markdown.markdown(md_content)
        # Extract text from HTML
        soup = BeautifulSoup(html, 'html.parser')
        return soup.get_text()

    def _process_single_chapter(self, file_path: Path) -> List[Dict]:
        """Process a single chapter file and return chunks with metadata"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract plain text from markdown
        plain_text = self._extract_text_from_markdown(content)

        # Create chunks
        chunks = create_chunks(plain_text, CHUNK_SIZE, CHUNK_OVERLAP)

        # Prepare chunk data with metadata
        chapter_chunks = []
        for i, chunk in enumerate(chunks):
            metadata = {
                'source_file': file_path.name,
                'chapter_title': file_path.stem,
                'chunk_index': i,
                'total_chunks': len(chunks),
                'content': chunk  # Store original content for retrieval
            }
            chapter_chunks.append({
                'text': chunk,
                'metadata': metadata
            })

        return chapter_chunks

    async def process_all_chapters(self, docs_dir: str = "../../docs"):
        """Process all chapter files in the docs directory"""
        docs_path = Path(docs_dir)
        if not docs_path.exists():
            raise FileNotFoundError(f"Docs directory does not exist: {docs_path}")

        # Get all markdown files
        chapter_files = list(docs_path.glob("*.md"))
        logger.info(f"Found {len(chapter_files)} chapter files to process")

        all_chunks = []
        for file_path in chapter_files:
            logger.info(f"Processing {file_path.name}")
            chunks = self._process_single_chapter(file_path)
            all_chunks.extend(chunks)
            logger.info(f"Processed {file_path.name} into {len(chunks)} chunks")

        # Process all chunks and add to vector database
        await self._add_chunks_to_vector_db(all_chunks)

        logger.info(f"Successfully processed {len(all_chunks)} total chunks from {len(chapter_files)} chapters")

    async def _add_chunks_to_vector_db(self, chunks: List[Dict]):
        """Add chunks to the vector database with embeddings"""
        if not chunks:
            logger.warning("No chunks to add to vector database")
            return

        # Prepare texts for embedding
        texts = [chunk['text'] for chunk in chunks]
        logger.info(f"Generating embeddings for {len(texts)} chunks...")

        # Get embeddings (in batches to avoid rate limits)
        embeddings = await get_embeddings(texts)

        # Prepare points for Qdrant
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point = models.PointStruct(
                id=str(i),  # Using index as ID, in production you might want UUIDs
                vector=embedding,
                payload={
                    "content": chunk['text'],
                    **chunk['metadata']
                }
            )
            points.append(point)

        # Upload to Qdrant in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i+batch_size]
            self.qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=batch
            )
            logger.info(f"Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")

        logger.info(f"Successfully uploaded {len(points)} chunks to vector database")

async def main():
    """Main function to process all chapters"""
    processor = ChapterProcessor()
    await processor.process_all_chapters()

if __name__ == "__main__":
    asyncio.run(main())