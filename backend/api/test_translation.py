import asyncio
import os
from dotenv import load_dotenv

# Load environment
load_dotenv()
load_dotenv('../.env')

# Import after loading environment
from hybrid_translation_service import HybridTranslationService

async def test_service():
    print('Creating hybrid service...')
    service = HybridTranslationService()

    print('Google API key:', 'Available' if service.google_api_key else 'Not available')

    # Test the fallback method directly
    fallback_result = await service._fallback_translate('Hello, how are you?', 'ur')
    print('Fallback result:', repr(fallback_result))

    # Test the main translation method
    try:
        result = await service.translate_text('Hello, how are you?', 'ur', 'en')
        print('Translation result:', repr(result))
    except Exception as e:
        print(f'Translation error: {e}')
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_service())