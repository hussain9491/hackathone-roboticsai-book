from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
load_dotenv()

app = FastAPI(
    title="Unified Backend API",
    description="AI Chat + Urdu Translation",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


from app.agent import router as agent_router


app.include_router(agent_router, prefix="/agent")


@app.get("/")
def root():
    return {"message": "Unified Backend API running"}