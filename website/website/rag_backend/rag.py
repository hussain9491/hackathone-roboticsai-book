from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import numpy as np
from config import settings
import logging

logger = logging.getLogger(__name__)

class RAGSystem:
    def __init__(self):
        self.client = QdrantClient(
            host=settings.qdrant_host,
            port=settings.qdrant_port,
            api_key=settings.qdrant_api_key
        )
        self.encoder = SentenceTransformer(settings.embedding_model_name)
        self.collection_name = settings.qdrant_collection_name

        # Initialize the collection if it doesn't exist
        self._initialize_collection()

    def _initialize_collection(self):
        """Initialize the Qdrant collection for storing textbook content"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.encoder.get_sentence_embedding_dimension(),
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")

                # Add some initial data for the Physical AI & Humanoid Robotics textbook
                self._add_initial_textbook_content()
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error initializing collection: {str(e)}")
            raise

    def _add_initial_textbook_content(self):
        """Add initial textbook content to the vector database"""
        # This is where we would load the actual textbook content
        # For now, we'll add some sample content related to the textbook topics

        textbook_content = [
            {
                "id": 1,
                "title": "Introduction to Physical AI",
                "content": "Physical AI represents the integration of artificial intelligence with physical systems, enabling robots to interact with the real world. Unlike traditional AI systems that operate in digital environments, physical AI systems must understand and navigate the complexities of physics, materials, and real-world constraints.",
                "source": "chapter1/section1-digital-ai-to-physical-ai",
                "metadata": {"chapter": 1, "section": 1, "topic": "Physical AI Fundamentals"}
            },
            {
                "id": 2,
                "title": "Humanoid Robot Design",
                "content": "Humanoid robots are designed with human-like form factors to enable interaction in human environments. Key considerations include bipedal locomotion, dexterous manipulation, and social interaction capabilities. The design must balance anthropomorphic features with functional requirements.",
                "source": "chapter1/section2-humanoid-form-factor",
                "metadata": {"chapter": 1, "section": 2, "topic": "Humanoid Design"}
            },
            {
                "id": 3,
                "title": "Safety in Robotics",
                "content": "Safety is paramount in physical AI systems. This includes both physical safety (preventing harm to humans and property) and ethical safety (ensuring appropriate behavior). Safety systems must be fail-safe and incorporate multiple layers of protection.",
                "source": "chapter1/section3-safety-ethics",
                "metadata": {"chapter": 1, "section": 3, "topic": "Robot Safety"}
            },
            {
                "id": 4,
                "title": "Embodied Cognition",
                "content": "Embodied cognition suggests that intelligence emerges from the interaction between an agent's body, its sensors and actuators, and the environment. This perspective has profound implications for robotics, emphasizing the importance of physical embodiment in intelligent behavior.",
                "source": "chapter1/section4-embodied-cognition",
                "metadata": {"chapter": 1, "section": 4, "topic": "Embodied Intelligence"}
            },
            {
                "id": 5,
                "title": "Physical Reasoning",
                "content": "Physical reasoning is the ability to understand and predict the behavior of objects and systems in the physical world. For robots operating in human environments, physical reasoning is crucial for safe and effective interaction with the world.",
                "source": "chapter1/section5-physical-reasoning",
                "metadata": {"chapter": 1, "section": 5, "topic": "Physical Reasoning"}
            },
            {
                "id": 6,
                "title": "Sensors in Robotics",
                "content": "Sensors form the foundation of a robot's ability to perceive and understand its environment. LiDAR and cameras are particularly important for humanoid robots, providing complementary capabilities in spatial and visual information.",
                "source": "chapter2/section1-sensors-lidar-cameras",
                "metadata": {"chapter": 2, "section": 1, "topic": "Robot Sensors"}
            },
            {
                "id": 7,
                "title": "Robot Actuators",
                "content": "Actuators are the muscles of robotic systems, converting electrical, hydraulic, or pneumatic energy into mechanical motion. In humanoid robots, actuators must replicate the complex, coordinated movements of human joints.",
                "source": "chapter2/section2-actuators",
                "metadata": {"chapter": 2, "section": 2, "topic": "Robot Actuators"}
            },
            {
                "id": 8,
                "title": "Robot Computing Platforms",
                "content": "The computational requirements of physical AI systems are substantial. Platforms like NVIDIA Jetson and RTX provide the necessary processing power while managing constraints like power consumption, size, and heat dissipation.",
                "source": "chapter2/section3-compute-jetson-rtx",
                "metadata": {"chapter": 2, "section": 3, "topic": "Robot Computing"}
            },
            {
                "id": 9,
                "title": "ROS 2 Architecture",
                "content": "ROS 2 represents a fundamental shift from ROS 1, designed specifically to address the needs of modern robotics applications. Its distributed architecture provides improved real-time capabilities, enhanced security, and better support for commercial deployment.",
                "source": "chapter3/section1-architecture",
                "metadata": {"chapter": 3, "section": 1, "topic": "ROS 2 Architecture"}
            },
            {
                "id": 10,
                "title": "ROS 2 Nodes",
                "content": "Nodes represent the fundamental building blocks of ROS 2 applications, serving as independent processes that perform specific functions within a robotic system. Understanding node architecture is essential for developing robust robotic systems.",
                "source": "chapter3/section2-nodes",
                "metadata": {"chapter": 3, "section": 2, "topic": "ROS 2 Nodes"}
            }
        ]

        # Encode the content and store in Qdrant
        points = []
        for item in textbook_content:
            # Encode the content text
            vector = self.encoder.encode(item["content"]).tolist()

            points.append(
                models.PointStruct(
                    id=item["id"],
                    vector=vector,
                    payload={
                        "title": item["title"],
                        "content": item["content"],
                        "source": item["source"],
                        "metadata": item["metadata"]
                    }
                )
            )

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        logger.info(f"Added {len(textbook_content)} textbook entries to Qdrant collection")

    def retrieve_relevant_content(self, query: str, top_k: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Retrieve the most relevant content from the vector database based on the query
        """
        if top_k is None:
            top_k = settings.top_k_retrieval

        # Encode the query
        query_vector = self.encoder.encode(query).tolist()

        # Search in Qdrant
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k,
            score_threshold=settings.similarity_threshold
        )

        # Extract relevant content
        results = []
        for result in search_results:
            results.append({
                "content": result.payload["content"],
                "title": result.payload["title"],
                "source": result.payload["source"],
                "metadata": result.payload["metadata"],
                "similarity_score": result.score
            })

        return results

    def generate_response(self, query: str, retrieved_content: List[Dict[str, Any]]) -> str:
        """
        Generate a response based on the query and retrieved content
        This is a simplified implementation - in practice, you'd use an LLM
        """
        if not retrieved_content:
            return "I couldn't find relevant information to answer your query about: " + query

        # Combine retrieved content to form context
        context_parts = []
        for item in retrieved_content:
            context_parts.append(f"Title: {item['title']}\nContent: {item['content']}")

        context = "\n\n".join(context_parts)

        # Generate a response based on the context and query
        # This is a simplified implementation - in a real system, you'd use an LLM
        response = f"Based on the Physical AI & Humanoid Robotics textbook, here's information about '{query}':\n\n"

        # Add the most relevant content
        most_relevant = retrieved_content[0]
        response += f"From {most_relevant['title']}:\n{most_relevant['content']}"

        # Add reference to source
        response += f"\n\nSource: {most_relevant['source']}"

        return response

# Global RAG system instance
rag_system = None

def get_rag_system() -> RAGSystem:
    """Get the global RAG system instance, creating it if needed"""
    global rag_system
    if rag_system is None:
        rag_system = RAGSystem()
    return rag_system