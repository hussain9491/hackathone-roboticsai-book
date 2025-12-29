# Quickstart: AI Study Assistant

## Prerequisites
- Node.js 20+ (for Docusaurus frontend)
- Python 3.10+ (for FastAPI backend)
- Access to Google Gemini API
- Access to Cohere embedding API
- Qdrant vector database instance

## Setup

### Backend Setup
1. Navigate to `website/website/backend/`
2. Install Python dependencies:
   ```bash
   pip install uv  # if not already installed
   uv sync
   ```
3. Set up environment variables in `.env`:
   ```
   GEMINI_API_KEY=your_gemini_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_KEY=your_qdrant_key
   ```
4. Start the backend server:
   ```bash
   uv run app.main:app --reload
   ```

### Frontend Setup
1. Navigate to `website/website/`
2. Install dependencies:
   ```bash
   npm install
   ```
3. Start the Docusaurus development server:
   ```bash
   npm start
   ```

## API Endpoints
- Backend API: `http://localhost:8000`
- Chat endpoint: `POST /agent/chat`
- Frontend connects to backend at `/agent/chat`

## Configuration
The AI Study Assistant is automatically integrated into all pages via the custom Layout override in `src/theme/Layout.tsx`. No additional configuration needed.

## Development
- The chat component is in `src/components/AIStudyAssistant.tsx`
- State management uses React hooks (useState, useEffect)
- Animations powered by Framer Motion
- Context (page title, URL) is automatically captured and sent with each message