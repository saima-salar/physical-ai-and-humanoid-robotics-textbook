import aiohttp
import logging
from config import OPENROUTER_API_KEY

logger = logging.getLogger(__name__)

class TranslationService:
    def __init__(self):
        # Initialize with OpenRouter API key
        if OPENROUTER_API_KEY:
            self.openrouter_api_key = OPENROUTER_API_KEY
            self.openrouter_available = True
        else:
            self.openrouter_available = False

    async def translate_text(self, text: str, target_language: str = "ur", source_language: str = "en") -> str:
        """
        Translate text using OpenRouter's language model
        """
        if not self.openrouter_available:
            raise Exception("OpenRouter API key not configured")

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
            logger.error(f"Translation error: {str(e)}")
            raise e

# Global instance
translation_service = TranslationService()