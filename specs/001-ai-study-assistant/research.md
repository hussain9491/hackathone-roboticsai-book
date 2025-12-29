# Research: AI Study Assistant Implementation

## Decision: Technology Stack
- **Frontend**: React/TypeScript with Docusaurus framework
- **UI Animation**: Framer Motion for smooth transitions
- **Styling**: Tailwind CSS with custom dark theme
- **Backend**: FastAPI with async support
- **AI Provider**: Google Gemini via OpenAI-compatible endpoint
- **Vector DB**: Qdrant for document retrieval
- **Embeddings**: Cohere embedding models

## Rationale:
The existing infrastructure provides a solid foundation for the AI Study Assistant. The backend already has a `/agent/chat` endpoint that accepts messages and returns AI responses with context from the textbook content. The frontend component is already scaffolded with proper state management and UI.

## Alternatives considered:
1. **Different AI providers**: OpenAI, Anthropic - settled on Gemini via OpenAI-compatible endpoint for cost/performance
2. **Different vector DBs**: Pinecone, Weaviate - Qdrant chosen for existing setup
3. **Different frontend frameworks**: Vue, Svelte - React chosen as it's already in use with Docusaurus

## Integration approach:
- Backend API is already available at `/agent/chat`
- Frontend component connects to backend via fetch API
- Context (page title, URL) is passed to backend for RAG functionality
- Component is mounted globally via custom Layout override