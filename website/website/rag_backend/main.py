from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
import uvicorn
import logging
from contextlib import asynccontextmanager
from rag import get_rag_system
from config import settings
from auth import auth_system, UserRegistration, UserLogin, Token, get_token_from_header

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Message(BaseModel):
    role: str  # "user" or "assistant"
    content: str

class ChatRequest(BaseModel):
    messages: List[Message]
    query: str
    max_tokens: Optional[int] = 500
    temperature: Optional[float] = 0.7

class ChatResponse(BaseModel):
    response: str
    sources: List[str]
    tokens_used: int

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Initializing RAG chatbot backend...")
    # Initialize the RAG system
    rag_system = get_rag_system()
    yield
    # Shutdown
    logger.info("Shutting down RAG chatbot backend...")

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot",
    description="A RAG-based chatbot for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan
)

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics RAG Chatbot API"}

@app.post("/auth/register", response_model=Token)
async def register(user_data: UserRegistration):
    """Register a new user with hardware background information"""
    try:
        # Validate email format
        if "@" not in user_data.email or "." not in user_data.email:
            raise HTTPException(status_code=400, detail="Invalid email format")

        # Validate password length
        if len(user_data.password) < 6:
            raise HTTPException(status_code=400, detail="Password must be at least 6 characters")

        # Register the user
        token = auth_system.register_user(user_data)
        return token
    except Exception as e:
        logger.error(f"Error registering user: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error registering user: {str(e)}")

@app.post("/auth/login", response_model=Token)
async def login(user_data: UserLogin):
    """Authenticate a user and return token"""
    try:
        # Authenticate the user
        token = auth_system.authenticate_user(user_data.email, user_data.password)
        return token
    except Exception as e:
        logger.error(f"Error authenticating user: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error authenticating user: {str(e)}")

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest, authorization: str = Depends(get_token_from_header)):
    """
    Main chat endpoint that processes user queries using RAG
    Requires authentication token
    """
    try:
        # Decode the token to get user information
        user_payload = auth_system.decode_access_token(authorization)
        user_hardware_background = user_payload.get("hardware_background", "")

        # Process the query using RAG
        response = await process_rag_query(request.query, request.messages, user_hardware_background)
        return ChatResponse(
            response=response,
            sources=response.get("sources", []) if isinstance(response, dict) else extract_sources(response),
            tokens_used=len(response.split()) if isinstance(response, str) else len(response.get("response", "").split())
        )
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

def extract_sources(response_text: str) -> List[str]:
    """
    Extract source information from response text
    """
    # This is a simple implementation - in practice, you'd have a more sophisticated way
    # to track which sources were used in the response
    sources = []
    if "chapter1" in response_text.lower():
        sources.append("Chapter 1: The Era of Embodied Intelligence")
    if "chapter2" in response_text.lower():
        sources.append("Chapter 2: The Hardware Nervous System")
    if "chapter3" in response_text.lower():
        sources.append("Chapter 3: ROS 2 Foundations")
    return sources

async def process_rag_query(query: str, conversation_history: List[Message], user_hardware_background: str = "") -> str:
    """
    Process a query using RAG (Retrieval Augmented Generation)
    """
    try:
        # Get the RAG system instance
        rag_system = get_rag_system()

        # Retrieve relevant content from the vector database
        retrieved_content = rag_system.retrieve_relevant_content(query)

        # Generate response based on retrieved content
        response = rag_system.generate_response(query, retrieved_content)

        # If user has hardware background, personalize the response
        if user_hardware_background:
            # Add personalized content based on user's hardware background
            response += f"\n\nNote: As you have a {user_hardware_background}, this information is particularly relevant to your setup."

        return response
    except Exception as e:
        logger.error(f"Error in RAG processing: {str(e)}")
        raise

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "RAG Chatbot Backend"}

@app.get("/collections")
async def list_collections():
    """List available collections in Qdrant"""
    try:
        rag_system = get_rag_system()
        collections = rag_system.client.get_collections()
        return {"collections": [col.name for col in collections.collections]}
    except Exception as e:
        logger.error(f"Error listing collections: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error listing collections: {str(e)}")

@app.get("/search/{query}")
async def search_content(query: str):
    """Search for content in the vector database"""
    try:
        rag_system = get_rag_system()
        results = rag_system.retrieve_relevant_content(query, top_k=5)
        return {"results": results}
    except Exception as e:
        logger.error(f"Error searching content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error searching content: {str(e)}")

@app.get("/user/profile")
async def get_user_profile(authorization: str = Depends(get_token_from_header)):
    """Get user profile information including hardware background"""
    try:
        # Decode the token to get user information
        user_payload = auth_system.decode_access_token(authorization)
        user_id = user_payload.get("sub")
        hardware_background = user_payload.get("hardware_background", "")

        # In a real implementation, you would fetch user details from a database
        # For now, we return the information from the token
        return {
            "user_id": user_id,
            "hardware_background": hardware_background
        }
    except Exception as e:
        logger.error(f"Error fetching user profile: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error fetching user profile: {str(e)}")

if __name__ == "__main__":
    uvicorn.run(app, host=settings.app_host, port=settings.app_port)