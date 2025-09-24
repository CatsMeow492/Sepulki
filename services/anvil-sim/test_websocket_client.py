#!/usr/bin/env python3
"""
Test WebSocket Client for Enhanced Isaac Sim Stream
"""

import asyncio
import websockets
import json
import cv2
import numpy as np
import base64
from datetime import datetime

async def test_websocket_connection():
    """Test WebSocket connection to enhanced Isaac Sim stream"""
    uri = "ws://localhost:8765"
    
    try:
        print(f"🔌 Connecting to {uri}...")
        async with websockets.connect(uri) as websocket:
            print("✅ WebSocket connected!")
            
            # Send start stream message
            message = {
                "type": "start_video_stream",
                "session_id": "test_client"
            }
            await websocket.send(json.dumps(message))
            print("📤 Sent start stream message")
            
            # Listen for messages
            frame_count = 0
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    if data.get("type") == "connection_established":
                        print(f"✅ Connection established: {data.get('message')}")
                        print(f"📊 Isaac Sim status: {data.get('isaac_sim_status')}")
                        print(f"🎬 Stream type: {data.get('stream_type')}")
                    
                    elif data.get("type") == "video_stream_started":
                        print(f"🎬 Video stream started: {data.get('message')}")
                    
                    elif data.get("type") == "video_frame":
                        frame_count += 1
                        if frame_count % 10 == 0:
                            print(f"📹 Received frame {frame_count}: {data.get('width')}x{data.get('height')} from {data.get('source')}")
                        
                        # Test frame decoding
                        if frame_count == 1:
                            try:
                                frame_data = data.get('frame_data')
                                if frame_data:
                                    # Decode base64 to test
                                    img_data = base64.b64decode(frame_data)
                                    nparr = np.frombuffer(img_data, np.uint8)
                                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                                    if img is not None:
                                        print(f"✅ Successfully decoded frame: {img.shape}")
                                    else:
                                        print("❌ Failed to decode frame")
                            except Exception as e:
                                print(f"❌ Frame decoding error: {e}")
                    
                    if frame_count >= 30:  # Stop after 30 frames
                        break
                        
                except json.JSONDecodeError:
                    print(f"❌ Invalid JSON: {message}")
                except Exception as e:
                    print(f"❌ Message processing error: {e}")
            
            print(f"🎉 Test completed! Received {frame_count} frames")
            
    except Exception as e:
        print(f"❌ WebSocket connection failed: {e}")

if __name__ == "__main__":
    asyncio.run(test_websocket_connection())
