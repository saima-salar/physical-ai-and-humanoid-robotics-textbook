import requests
import json

def test_chat_api():
    """Test the chat API endpoint"""
    url = "http://localhost:8000/chat"

    # Sample request
    payload = {
        "message": "What is Physical AI?",
        "selected_text": "Physical AI represents a transformative frontier in artificial intelligence, extending its capabilities beyond the digital realm into the tangible world."
    }

    try:
        response = requests.post(url, json=payload)
        if response.status_code == 200:
            data = response.json()
            print("✓ Chat API test successful!")
            print(f"Response: {data['response'][:100]}...")
            return True
        else:
            print(f"✗ Chat API returned status code: {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Chat API is not running. Please start the backend first.")
        return False
    except Exception as e:
        print(f"✗ Error testing Chat API: {str(e)}")
        return False

def test_health_api():
    """Test the health API endpoint"""
    url = "http://localhost:8000/health"

    try:
        response = requests.get(url)
        if response.status_code == 200:
            print("✓ Health API test successful!")
            return True
        else:
            print(f"✗ Health API returned status code: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Health API is not running. Please start the backend first.")
        return False

def main():
    print("Testing RAG Chatbot API Endpoints")
    print("=" * 40)

    # Test health endpoint
    health_ok = test_health_api()
    print()

    # Test chat endpoint
    chat_ok = test_chat_api()
    print()

    print("API Test Summary:")
    print(f"- Health API: {'✓ Passed' if health_ok else '✗ Failed'}")
    print(f"- Chat API: {'✓ Passed' if chat_ok else '✗ Failed'}")

if __name__ == "__main__":
    main()