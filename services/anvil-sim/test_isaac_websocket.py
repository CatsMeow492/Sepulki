#!/usr/bin/env python3
"""
Test Isaac Sim WebSocket Livestream Connection
"""

import websocket
import json
import time
import threading

def test_isaac_websocket():
    """Test Isaac Sim WebSocket livestream connection"""
    
    def on_message(ws, message):
        print(f"📨 Received message: {message[:100]}...")
    
    def on_error(ws, error):
        print(f"❌ WebSocket error: {error}")
    
    def on_close(ws, close_status_code, close_msg):
        print("🔌 WebSocket closed")
    
    def on_open(ws):
        print("✅ WebSocket connected to Isaac Sim")
        
        # Send a test message
        test_message = {
            "type": "ping",
            "timestamp": time.time()
        }
        ws.send(json.dumps(test_message))
        
        # Send a request for livestream status
        status_message = {
            "type": "get_status",
            "timestamp": time.time()
        }
        ws.send(json.dumps(status_message))
    
    # Test WebSocket connection
    print("🔌 Testing WebSocket connection to Isaac Sim...")
    
    try:
        ws_url = "ws://localhost:49100"
        ws = websocket.WebSocketApp(ws_url,
                                  on_open=on_open,
                                  on_message=on_message,
                                  on_error=on_error,
                                  on_close=on_close)
        
        # Run for 10 seconds
        def timeout_handler():
            time.sleep(10)
            ws.close()
        
        timeout_thread = threading.Thread(target=timeout_handler)
        timeout_thread.daemon = True
        timeout_thread.start()
        
        ws.run_forever()
        
    except Exception as e:
        print(f"❌ WebSocket test failed: {e}")

if __name__ == "__main__":
    test_isaac_websocket()
