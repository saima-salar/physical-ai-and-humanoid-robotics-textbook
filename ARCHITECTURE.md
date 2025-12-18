# Physical AI and Humanoid Robotics - Architecture

## Overview

This project implements a textbook website with an integrated RAG (Retrieval-Augmented Generation) chatbot, organized as a monorepo with separate backend and frontend applications.

## Directory Structure

```
physical-ai-and-humanoid-robotics/
├── backend/                    # FastAPI backend services
│   ├── api/                    # Main API application (FastAPI)
│   │   ├── main.py            # Main application entry point
│   │   ├── config.py          # Configuration management
│   │   ├── models/            # Pydantic models
│   │   └── services/          # Business logic (RAG, conversation)
│   ├── docs_processing/       # Document processing scripts
│   ├── requirements.txt       # Python dependencies
│   └── .env                   # Environment variables
├── frontend/                   # Docusaurus frontend application
│   ├── docs/                  # Textbook content (Markdown)
│   ├── src/                   # Source components
│   │   ├── components/        # React components (including Chatbot)
│   │   ├── theme/             # Docusaurus theme customization
│   │   └── pages/             # Static pages
│   ├── static/                # Static assets
│   ├── package.json           # Node dependencies
│   └── docusaurus.config.js   # Docusaurus configuration
├── docker-compose.yml         # Docker orchestration
├── Dockerfile                 # Root Dockerfile (builds both)
├── package.json               # Root package.json (monorepo scripts)
└── README.md                  # Project overview
```

## Backend Services (Python/FastAPI)

### Core Components
- **Main API** (`backend/api/`): FastAPI application handling chat requests
- **RAG Service** (`backend/api/services/rag_service.py`): Semantic search and response generation
- **Conversation Service** (`backend/api/services/conversation_service.py`): History management
- **Document Processing** (`backend/docs_processing/`): Textbook content indexing

### Technologies
- FastAPI: Web framework
- OpenAI: Embeddings and language models
- Qdrant: Vector database for semantic search
- SQLAlchemy: Database ORM
- Neon Postgres: Conversation storage

## Frontend Application (Docusaurus/React)

### Core Components
- **Textbook Content** (`frontend/docs/`): Markdown files for textbook chapters
- **Chatbot UI** (`frontend/src/components/Chatbot*`): Interactive chat interface
- **Personalization** (`frontend/src/components/Personalization*`): User preference system
- **Layout Integration** (`frontend/src/theme/Layout.js`): Chatbot injection

### Technologies
- Docusaurus: Static site generator
- React: UI components
- KaTeX: Mathematical expression rendering

## Development Workflow

### Local Development

1. **Start Backend**:
   ```bash
   cd backend/api
   pip install -r ../requirements.txt
   uvicorn main:app --reload
   ```

2. **Start Frontend**:
   ```bash
   cd frontend
   npm install
   npm run start
   ```

3. **Or use root scripts**:
   ```bash
   npm run dev  # Runs both concurrently
   ```

### Environment Variables

#### Backend (.env in backend/)
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_db_url
```

#### Frontend (via docusaurus.config.js)
```javascript
customFields: {
  BACKEND_API_URL: process.env.BACKEND_API_URL || 'http://localhost:8000'
}
```

## Deployment

### Docker Compose
```bash
docker-compose up -d
```

### Manual Deployment
1. Build frontend: `cd frontend && npm run build`
2. Deploy backend: Deploy FastAPI app to preferred platform
3. Configure environment variables

## API Endpoints

### Backend API
- `GET /health`: Health check
- `POST /chat`: Chat endpoint
- `GET /conversations/{id}`: Conversation history

### Frontend Integration
- Chatbot widget appears on all textbook pages
- Supports text selection for contextual questions
- Maintains conversation history
- Shows source attribution

## Key Features

1. **RAG Chatbot**: Answers questions based on textbook content
2. **Text Selection Context**: Ask questions about selected text
3. **Personalization**: Adapts to user preferences and skill level
4. **Source Attribution**: Shows where answers come from
5. **Responsive Design**: Works on all devices
6. **Math Rendering**: Supports mathematical expressions
7. **Content Filtering**: Personalized content views