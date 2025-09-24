#!/usr/bin/env python3
"""
Test Isaac Sim Livestream Connection
"""

import requests
import json
import time

def test_isaac_livestream():
    """Test Isaac Sim livestream endpoints"""
    
    base_url = "http://localhost:49100"
    
    # Test various endpoints
    endpoints = [
        "/",
        "/livestream",
        "/livestream/status", 
        "/webrtc",
        "/api/livestream",
        "/api/status",
        "/stream",
        "/video",
        "/rtc",
        "/ws",
        "/websocket"
    ]
    
    print("🔍 Testing Isaac Sim livestream endpoints...")
    
    for endpoint in endpoints:
        try:
            url = f"{base_url}{endpoint}"
            response = requests.get(url, timeout=5)
            print(f"✅ {endpoint}: {response.status_code}")
            
            if response.status_code != 501:
                print(f"   Headers: {dict(response.headers)}")
                if response.text:
                    print(f"   Content: {response.text[:200]}")
                    
        except requests.exceptions.RequestException as e:
            print(f"❌ {endpoint}: {e}")
    
    # Test WebSocket connection
    print("\n🔌 Testing WebSocket connection...")
    try:
        import websocket
        
        def on_message(ws, message):
            print(f"📨 WebSocket message: {message[:100]}...")
        
        def on_error(ws, error):
            print(f"❌ WebSocket error: {error}")
        
        def on_close(ws, close_status_code, close_msg):
            print("🔌 WebSocket closed")
        
        def on_open(ws):
            print("✅ WebSocket connected")
            ws.send(json.dumps({"type": "get_status"}))
        
        ws_url = "ws://localhost:49100/ws"
        ws = websocket.WebSocketApp(ws_url,
                                  on_open=on_open,
                                  on_message=on_message,
                                  on_error=on_error,
                                  on_close=on_close)
        
        ws.run_forever(timeout=5)
        
    except ImportError:
        print("❌ websocket-client not installed")
    except Exception as e:
        print(f"❌ WebSocket test failed: {e}")

if __name__ == "__main__":
    test_isaac_livestream()
