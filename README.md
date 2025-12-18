# Physical AI and Humanoid Robotics Textbook with Integrated RAG Chatbot

This project implements a comprehensive textbook website with an integrated RAG (Retrieval-Augmented Generation) chatbot, organized as a monorepo with separate backend and frontend applications.

## 🏗️ Architecture Overview

**Backend**: FastAPI application handling RAG functionality, document processing, and conversation management
**Frontend**: Docusaurus-based textbook website with integrated chatbot widget
**Database**: Qdrant vector database for semantic search, Neon Postgres for conversation history

For detailed architecture information, see [ARCHITECTURE.md](./ARCHITECTURE.md).

## 📁 Directory Structure

```
physical-ai-and-humanoid-robotics/
├── backend/                    # FastAPI backend services
│   ├── api/                    # Main API application
│   ├── docs_processing/        # Document processing scripts
│   ├── requirements.txt        # Python dependencies
│   └── .env                    # Environment variables
├── frontend/                   # Docusaurus frontend application
│   ├── docs/                   # Textbook content (Markdown)
│   ├── src/                    # Source components
│   ├── static/                 # Static assets
│   ├── package.json            # Node dependencies
│   └── docusaurus.config.js    # Docusaurus configuration
├── docker-compose.yml          # Docker orchestration
└── ARCHITECTURE.md             # Detailed architecture documentation
```

## 🚀 Quick Start

### Prerequisites
- Node.js 18+
- Python 3.11+
- pip
- npm

### Development Setup

1. **Install dependencies**:
   ```bash
   npm run setup  # Installs dependencies for both frontend and backend
   ```

2. **Set up environment variables**:
   - Copy `backend/.env` and update with your API keys
   - Required: OpenAI API Key, Qdrant URL & Key, Neon Database URL

3. **Run development servers**:
   ```bash
   npm run dev  # Runs both frontend and backend concurrently
   ```

4. **Access the applications**:
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000

### Manual Setup

**Backend**:
```bash
cd backend
pip install -r requirements.txt
cd api
uvicorn main:app --reload
```

**Frontend**:
```bash
cd frontend
npm install
npm run start
```

## ⚙️ Features

- **Textbook Content**: Comprehensive Physical AI and Humanoid Robotics content
- **RAG Chatbot**: Ask questions about textbook content with accurate responses
- **Text Selection**: Ask questions about specific selected text for contextual answers
- **Personalization**: Adaptive content based on user preferences and skill level
- **Math Rendering**: Properly formatted mathematical expressions
- **Source Attribution**: Shows where answers come from in the textbook
- **Responsive Design**: Works on all devices

## 🛠️ API Endpoints (Backend)

- `GET /health` - Health check
- `POST /chat` - Chat with RAG system
- `GET /conversations/{id}` - Get conversation history

## 📦 Deployment

### Docker Compose
```bash
docker-compose up -d
```

### Manual Deployment
1. Build frontend: `cd frontend && npm run build`
2. Deploy backend: Deploy FastAPI application
3. Configure environment variables
4. Set up domain routing (frontend on root, backend on /api or subdomain)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is licensed under [MIT License](LICENSE).