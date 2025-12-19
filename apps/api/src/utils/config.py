import os
from dotenv import load_dotenv

def validate_config():
    """
    Validates that all required environment variables are set.
    """
    load_dotenv()
    
    required_vars = [
        "OPENAI_API_KEY",
        "GEMINI_API_KEY",
        "COHERE_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY"
    ]
    
    missing = [var for var in required_vars if not os.getenv(var)]
    
    if missing:
        raise EnvironmentError(f"Missing required environment variables: {', '.join(missing)}")
    
    print("Environment configuration validated successfully.")

if __name__ == "__main__":
    validate_config()
