import logging
import requests
import os
import aiohttp
from typing import Optional
from dotenv import load_dotenv
from config import OPENROUTER_API_KEY

# Load environment variables
load_dotenv()
load_dotenv('../.env')

logger = logging.getLogger(__name__)

class HybridTranslationService:
    def __init__(self):
        # Google Translate configuration
        self.google_api_key = os.getenv('GOOGLE_TRANSLATE_API_KEY')
        self.google_base_url = "https://translation.googleapis.com/language/translate/v2"

        # Google Gemini configuration
        self.gemini_api_key = os.getenv('GEMINI_API_KEY')
        self.gemini_available = bool(self.gemini_api_key)

        # OpenRouter configuration
        self.openrouter_api_key = OPENROUTER_API_KEY
        self.openrouter_available = bool(OPENROUTER_API_KEY)

    async def translate_text(self, text: str, target_language: str = "ur", source_language: str = "en") -> str:
        """
        Translate text using multiple fallback methods:
        1. Google Translate API (if configured and available)
        2. Google Gemini API (if configured and available)
        3. OpenAI API (if configured and available)
        4. Fallback translation (basic method)
        """
        # Method 1: Try Google Translate API
        if self.google_api_key:
            try:
                result = await self._google_translate(text, target_language, source_language)
                if result and "TRANSLATION SERVICE NOT CONFIGURED" not in result:
                    return result
            except Exception as e:
                logger.warning(f"Google Translate failed: {e}")

        # Method 2: Try Google Gemini API
        if self.gemini_available:
            try:
                result = await self._gemini_translate(text, target_language, source_language)
                if result and "TRANSLATION SERVICE NOT CONFIGURED" not in result:
                    return result
            except Exception as e:
                logger.warning(f"Gemini translation failed: {e}")

        # Method 3: Try OpenRouter API
        if self.openrouter_available:
            try:
                result = await self._openrouter_translate(text, target_language, source_language)
                if result:
                    return result
            except Exception as e:
                logger.warning(f"OpenRouter translation failed: {e}")

        # Method 4: Fallback to basic translation
        logger.info("Using fallback translation method")
        return await self._fallback_translate(text, target_language)

    async def _google_translate(self, text: str, target_language: str, source_language: str) -> str:
        """Translate using Google Cloud Translation API"""
        if not self.google_api_key:
            raise Exception("Google Translate API key not configured")

        try:
            url = f"{self.google_base_url}?key={self.google_api_key}"

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
                logger.error(f"Google Translation API error: {response.status_code} - {response.text}")
                # Return fallback if Google API fails
                return await self._fallback_translate(text, target_language)
        except Exception as e:
            logger.error(f"Google Translation error: {str(e)}")
            # Return fallback if Google API fails
            return await self._fallback_translate(text, target_language)

    async def _openrouter_translate(self, text: str, target_language: str, source_language: str) -> str:
        """Translate using OpenRouter's language model"""
        if not self.openrouter_available:
            raise Exception("OpenRouter not configured")

        try:
            prompt = f"""
            Translate the following text from {source_language} to {target_language} (Urdu).
            Preserve the technical terminology and meaning as accurately as possible.
            Return only the translated text without any additional commentary.

            Text to translate:
            {text}
            """

            url = "https://openrouter.ai/api/v1/chat/completions"

            headers = {
                "Authorization": f"Bearer {self.openrouter_api_key}",
                "Content-Type": "application/json"
            }

            payload = {
                "model": "google/gemma-7b-it",  # Using a model that supports translation
                "messages": [
                    {
                        "role": "system",
                        "content": "You are a professional translator specializing in technical content. Translate the given text accurately while preserving technical terminology."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                "max_tokens": min(len(text) * 2, 4000),  # Limit to avoid token overflow
                "temperature": 0.3
            }

            async with aiohttp.ClientSession() as session:
                async with session.post(url, headers=headers, json=payload) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"OpenRouter Translation API error {response.status}: {error_text}")
                        raise Exception(f"OpenRouter API error: {error_text}")

                    data = await response.json()
                    translated_text = data['choices'][0]['message']['content'].strip()
                    return translated_text
        except Exception as e:
            logger.error(f"OpenRouter Translation error: {str(e)}")
            raise e

    async def _gemini_translate(self, text: str, target_language: str, source_language: str) -> str:
        """Translate using Google Gemini API"""
        if not self.gemini_available:
            raise Exception("Gemini API not configured")

        try:
            prompt = f"""
            Translate the following text from {source_language} to {target_language} (Urdu).
            Preserve the technical terminology and meaning as accurately as possible.
            Return only the translated text without any additional commentary.

            Text to translate:
            {text}
            """

            url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent?key={self.gemini_api_key}"

            headers = {
                "Content-Type": "application/json"
            }

            payload = {
                "contents": [{
                    "parts": [{
                        "text": prompt
                    }]
                }],
                "generationConfig": {
                    "temperature": 0.3,
                    "maxOutputTokens": min(len(text) * 2, 8192)
                }
            }

            async with aiohttp.ClientSession() as session:
                async with session.post(url, headers=headers, json=payload) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logger.error(f"Gemini Translation API error {response.status}: {error_text}")
                        raise Exception(f"Gemini API error: {error_text}")

                    data = await response.json()

                    # Extract the translated text from Gemini's response
                    try:
                        translated_text = data['candidates'][0]['content']['parts'][0]['text'].strip()
                        return translated_text
                    except (KeyError, IndexError) as e:
                        logger.error(f"Error parsing Gemini response: {e}, response: {data}")
                        return await self._fallback_translate(text, target_language)
        except Exception as e:
            logger.error(f"Gemini Translation error: {str(e)}")
            raise e

    async def _fallback_translate(self, text: str, target_language: str) -> str:
        """
        Fallback translation method for when APIs are not available
        """
        # For Urdu specifically, provide a more helpful message
        if target_language.lower() == 'ur':
            # Return the original text with a note that translation is unavailable
            # Using English text to avoid encoding issues
            return f"Original: {text} [Urdu translation via HYBRID SERVICE - unavailable, please configure translation API]"
        else:
            return f"[TRANSLATION UNAVAILABLE via HYBRID SERVICE] Original: {text} [Please configure a translation API]"

# Global instance
translation_service = HybridTranslationService()