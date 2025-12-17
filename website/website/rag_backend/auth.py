from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
import jwt
import hashlib
import secrets
from datetime import datetime, timedelta
from config import settings
import logging

logger = logging.getLogger(__name__)

# User model
class UserRegistration(BaseModel):
    email: str
    password: str
    hardwareBackground: Optional[str] = None  # Hardware background information
    firstName: Optional[str] = None
    lastName: Optional[str] = None

class UserLogin(BaseModel):
    email: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str
    user_id: str
    hardware_background: Optional[str] = None

# Simple in-memory storage for development (would use database in production)
users_db = {}
sessions_db = {}

class AuthSystem:
    def __init__(self):
        self.secret_key = settings.qdrant_api_key or "fallback-secret-for-dev"
        self.algorithm = "HS256"
        self.access_token_expire_minutes = 30

    def hash_password(self, password: str) -> str:
        """Hash a password with a random salt"""
        salt = secrets.token_hex(16)
        pwdhash = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt.encode('utf-8'), 100000)
        return f"{salt}${pwdhash.hex()}"

    def verify_password(self, password: str, stored_hash: str) -> bool:
        """Verify a password against its hash"""
        try:
            salt, stored_pwdhash = stored_hash.split('$')
            pwdhash = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt.encode('utf-8'), 100000)
            return pwdhash.hex() == stored_pwdhash
        except:
            return False

    def create_access_token(self, user_id: str, hardware_background: Optional[str] = None):
        """Create a JWT access token"""
        expire = datetime.utcnow() + timedelta(minutes=self.access_token_expire_minutes)
        to_encode = {
            "sub": user_id,
            "exp": expire,
            "hardware_background": hardware_background
        }
        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)
        return encoded_jwt

    def decode_access_token(self, token: str):
        """Decode a JWT access token"""
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])
            return payload
        except jwt.ExpiredSignatureError:
            raise HTTPException(status_code=401, detail="Token has expired")
        except jwt.JWTError:
            raise HTTPException(status_code=401, detail="Could not validate credentials")

    def register_user(self, user_data: UserRegistration):
        """Register a new user with hardware background information"""
        email = user_data.email.lower().strip()

        # Check if user already exists
        if email in users_db:
            raise HTTPException(status_code=400, detail="User already registered")

        # Hash the password
        hashed_password = self.hash_password(user_data.password)

        # Create user record
        user = {
            "id": f"user_{secrets.token_hex(8)}",
            "email": email,
            "hashed_password": hashed_password,
            "hardware_background": user_data.hardwareBackground,
            "first_name": user_data.firstName,
            "last_name": user_data.lastName,
            "created_at": datetime.utcnow().isoformat(),
            "is_active": True
        }

        users_db[email] = user
        logger.info(f"User registered: {email} with hardware background: {user_data.hardwareBackground}")

        # Create access token
        access_token = self.create_access_token(user["id"], user_data.hardwareBackground)

        return Token(
            access_token=access_token,
            token_type="bearer",
            user_id=user["id"],
            hardware_background=user_data.hardwareBackground
        )

    def authenticate_user(self, email: str, password: str):
        """Authenticate a user and return token"""
        email = email.lower().strip()
        user = users_db.get(email)

        if not user or not self.verify_password(password, user["hashed_password"]):
            raise HTTPException(status_code=401, detail="Incorrect email or password")

        if not user["is_active"]:
            raise HTTPException(status_code=401, detail="User account is disabled")

        # Create access token
        access_token = self.create_access_token(user["id"], user.get("hardware_background"))

        return Token(
            access_token=access_token,
            token_type="bearer",
            user_id=user["id"],
            hardware_background=user.get("hardware_background")
        )

    def get_current_user(self, token: str = Depends(get_token_from_header)):
        """Get the current user from the token"""
        payload = self.decode_access_token(token)
        user_id = payload.get("sub")

        # Find user by ID
        user = None
        for email, u in users_db.items():
            if u["id"] == user_id:
                user = u
                break

        if user is None:
            raise HTTPException(status_code=401, detail="User not found")

        if not user["is_active"]:
            raise HTTPException(status_code=401, detail="User account is disabled")

        return user

# Global auth system instance
auth_system = AuthSystem()

# Helper function to get token from header
def get_token_from_header(authorization: str = None):
    if authorization is None:
        raise HTTPException(status_code=401, detail="Authorization header required")

    try:
        token_type, token = authorization.split()
        if token_type.lower() != "bearer":
            raise HTTPException(status_code=401, detail="Invalid token type")
        return token
    except ValueError:
        raise HTTPException(status_code=401, detail="Invalid authorization header format")