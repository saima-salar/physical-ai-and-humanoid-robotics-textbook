import logging
import requests
import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()
# Also try to load from parent directory
load_dotenv('../.env')

logger = logging.getLogger(__name__)

class FreeTranslationService:
    def __init__(self):
        # Google Cloud Translation API endpoint
        # Note: This uses the REST API directly without requiring a client library
        self.api_key = os.getenv('GOOGLE_TRANSLATE_API_KEY')  # Get from environment
        self.base_url = "https://translation.googleapis.com/language/translate/v2"

    def set_api_key(self, api_key: str):
        """Set the API key from environment or configuration"""
        self.api_key = api_key

    async def translate_text(self, text: str, target_language: str = "ur", source_language: str = "en") -> str:
        """
        Translate text using Google Cloud Translation API (free tier available)
        """
        if not self.api_key:
            # Fallback to a completely free service for basic functionality
            return await self.fallback_translate(text, target_language)

        try:
            url = f"{self.base_url}?key={self.api_key}"

            payload = {
                "q": text,
                "target": target_language,
                "source": source_language,
                "format": "text"
            }

            response = requests.post(url, json=payload)

            if response.status_code == 200:
                result = response.json()
                translated_text = result['data']['translations'][0]['translatedText']
                return translated_text
            else:
                logger.error(f"Translation API error: {response.status_code} - {response.text}")
                return await self.fallback_translate(text, target_language)

        except Exception as e:
            logger.error(f"Translation error: {str(e)}")
            return await self.fallback_translate(text, target_language)

    async def fallback_translate(self, text: str, target_language: str = "ur") -> str:
        """
        Fallback translation method for completely free usage
        This is a placeholder that returns the original text with a note
        """
        # In a real implementation, you could use a completely free service
        # or implement a simple dictionary-based translation for common terms
        return f"[TRANSLATION SERVICE NOT CONFIGURED] Original text: {text[:200]}... [Please configure a translation API]"


# Global instance
translation_service = FreeTranslationService()