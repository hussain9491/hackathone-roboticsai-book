import os
from typing import Optional
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # Qdrant Configuration
    qdrant_host: str = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port: int = int(os.getenv("QDRANT_PORT", 6333))
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "robotics_textbook")

    # Model Configuration
    embedding_model_name: str = os.getenv("EMBEDDING_MODEL_NAME", "all-MiniLM-L6-v2")
    llm_model_name: str = os.getenv("LLM_MODEL_NAME", "gpt-3.5-turbo")  # This would be replaced with local model in production

    # Application Configuration
    app_host: str = os.getenv("APP_HOST", "0.0.0.0")
    app_port: int = int(os.getenv("APP_PORT", 8000))
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"

    # RAG Configuration
    top_k_retrieval: int = int(os.getenv("TOP_K_RETRIEVAL", 5))
    similarity_threshold: float = float(os.getenv("SIMILARITY_THRESHOLD", 0.5))
    max_context_length: int = int(os.getenv("MAX_CONTEXT_LENGTH", 2000))

    class Config:
        env_file = ".env"

settings = Settings()