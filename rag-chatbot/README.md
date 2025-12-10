# RAG Chatbot for Physical AI and Humanoid Robotics Textbook

This RAG (Retrieval-Augmented Generation) chatbot allows users to ask questions about the content of the Physical AI and Humanoid Robotics textbook and get accurate answers based on the book's content.

## Architecture Overview

The system consists of the following components:

1. **Document Processing Pipeline**: Processes the textbook chapters and converts them into embeddings
2. **Vector Database**: Qdrant Cloud for storing document embeddings
3. **Backend API**: FastAPI service with OpenAI integration
4. **Frontend Interface**: Integration with the Docusaurus site
5. **Database**: Neon Serverless Postgres for storing conversation history

## Technology Stack

- **Backend**: FastAPI
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Database**: Neon Serverless Postgres
- **AI/ML**: OpenAI API
- **Frontend**: React components for Docusaurus integration
- **Document Processing**: Python with libraries like markdown, beautifulsoup4, etc.

## Features

- Answer questions based on textbook content
- Support for user-selected text context
- Conversation history tracking
- Source attribution for answers
- Real-time interaction
- Floating chatbot widget integrated into Docusaurus pages

## Project Structure

```
rag-chatbot/
├── backend/                 # FastAPI backend
│   ├── main.py             # Main API application
│   ├── config.py           # Configuration settings
│   ├── models/             # Data models
│   │   └── chat_models.py  # Chat request/response models
│   ├── services/           # Business logic
│   │   ├── rag_service.py  # RAG functionality
│   │   └── conversation_service.py # Conversation history
│   ├── requirements.txt    # Python dependencies
│   └── .env.example        # Environment variables template
├── docs-processing/        # Document processing scripts
│   ├── process_chapters.py # Chapter processing pipeline
│   └── embedding_utils.py  # Embedding utilities
├── src/plugins/            # Docusaurus plugins
│   └── docusaurus-plugin-chatbot/ # Chatbot integration
├── src/components/         # React components
│   ├── Chatbot.jsx         # Main chatbot component
│   ├── Chatbot.css         # Chatbot styles
│   └── ChatbotModal.jsx    # Modal wrapper for chatbot
├── setup_and_test.py       # Setup and test script
├── test_api.py             # API testing script
└── README.md              # This file
```

## Setup Instructions

### 1. Environment Configuration

1. Copy the environment template:
   ```bash
   cp rag-chatbot/.env.example rag-chatbot/.env
   ```

2. Update the `.env` file with your actual credentials:
   - `OPENAI_API_KEY`: Your OpenAI API key
   - `QDRANT_URL`: Your Qdrant cluster URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `NEON_DATABASE_URL`: Your Neon Postgres connection string

### 2. Backend Setup

1. Install Python dependencies:
   ```bash
   cd rag-chatbot/backend
   pip install -r requirements.txt
   ```

2. Process textbook chapters and populate vector database:
   ```bash
   cd rag-chatbot
   python setup_and_test.py
   ```

3. Start the backend API:
   ```bash
   cd rag-chatbot/backend
   uvicorn main:app --reload
   ```

### 3. Frontend Integration

The chatbot is automatically integrated into the Docusaurus site via a custom plugin. The floating chatbot widget will appear on all documentation pages.

To start the Docusaurus site:
```bash
npm run start
```

## Usage

1. Start both the backend API and the Docusaurus site
2. Navigate to any textbook chapter in your browser
3. Click the chatbot icon (💬) in the bottom-right corner
4. Ask questions about the textbook content
5. You can also select text on the page and ask questions about it for more specific context

## API Endpoints

- `GET /health`: Health check endpoint
- `POST /chat`: Main chat endpoint for asking questions
- `GET /conversations/{conversation_id}`: Get conversation history

## Testing

Run the API tests:
```bash
cd rag-chatbot
python test_api.py
```

## Architecture Details

### RAG Process
1. User asks a question
2. Question is embedded using OpenAI embeddings
3. Vector search is performed in Qdrant to find relevant textbook chunks
4. Relevant context is combined with the question and sent to OpenAI
5. OpenAI generates a response based on the textbook content
6. Response is returned to the user with source attribution

### Document Processing
- Textbook chapters are processed from the `docs/` directory
- Each chapter is split into overlapping chunks
- Chunks are embedded and stored in Qdrant vector database
- Metadata includes source file, chapter title, and chunk position