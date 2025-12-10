import asyncio
import os
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
load_dotenv('rag-chatbot/.env')

from docs-processing.process_chapters import ChapterProcessor

async def setup_rag_system():
    """Setup the RAG system by processing all textbook chapters"""
    print("Setting up RAG system...")

    # Check if required environment variables are set
    required_vars = ['OPENAI_API_KEY', 'QDRANT_URL']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"Error: Missing required environment variables: {missing_vars}")
        print("Please set them in rag-chatbot/.env file")
        return False

    try:
        # Process all chapters
        processor = ChapterProcessor()
        await processor.process_all_chapters(docs_dir="../docs")
        print("RAG system setup completed successfully!")
        return True
    except Exception as e:
        print(f"Error setting up RAG system: {str(e)}")
        return False

def test_backend():
    """Test the backend API"""
    import requests

    print("\nTesting backend API...")

    try:
        response = requests.get("http://localhost:8000/health")
        if response.status_code == 200:
            print("✓ Backend API is running and healthy")
            return True
        else:
            print(f"✗ Backend API returned status code: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Backend API is not running. Please start it with: cd rag-chatbot/backend && uvicorn main:app --reload")
        return False

def main():
    print("RAG Chatbot Setup and Test Script")
    print("=" * 40)

    # Setup RAG system
    success = asyncio.run(setup_rag_system())

    if success:
        # Test backend
        backend_ok = test_backend()

        print("\nSetup Summary:")
        print(f"- Document Processing: {'✓ Completed' if success else '✗ Failed'}")
        print(f"- Backend API Test: {'✓ Passed' if backend_ok else '✗ Failed'}")

        if backend_ok:
            print("\nTo start the backend API, run:")
            print("cd rag-chatbot/backend && uvicorn main:app --reload")

            print("\nTo start the Docusaurus site with the chatbot, run:")
            print("npm run start")

            print("\nThe RAG chatbot is now ready to use!")
        else:
            print("\nPlease ensure the backend API is running before using the chatbot.")
    else:
        print("\nSetup failed. Please check the error messages above.")

if __name__ == "__main__":
    main()