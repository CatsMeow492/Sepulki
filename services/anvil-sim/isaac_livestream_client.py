#!/usr/bin/env python3
"""
Isaac Sim Livestream Client
"""

import requests
import json
import time
import cv2
import numpy as np
import base64

def test_isaac_livestream_client():
    """Test Isaac Sim livestream client"""
    
    base_url = "http://localhost:49100"
    
    # Test different approaches
    print("🔍 Testing Isaac Sim livestream client...")
    
    # Try to get livestream info
    try:
        response = requests.get(f"{base_url}/info", timeout=5)
        print(f"✅ /info: {response.status_code}")
        if response.status_code == 200:
            print(f"   Content: {response.text}")
    except Exception as e:
        print(f"❌ /info: {e}")
    
    # Try to get livestream status
    try:
        response = requests.get(f"{base_url}/status", timeout=5)
        print(f"✅ /status: {response.status_code}")
        if response.status_code == 200:
            print(f"   Content: {response.text}")
    except Exception as e:
        print(f"❌ /status: {e}")
    
    # Try to get livestream stream
    try:
        response = requests.get(f"{base_url}/stream", timeout=5)
        print(f"✅ /stream: {response.status_code}")
        if response.status_code == 200:
            print(f"   Content type: {response.headers.get('content-type', 'unknown')}")
            print(f"   Content length: {len(response.content)}")
    except Exception as e:
        print(f"❌ /stream: {e}")
    
    # Try to get livestream video
    try:
        response = requests.get(f"{base_url}/video", timeout=5)
        print(f"✅ /video: {response.status_code}")
        if response.status_code == 200:
            print(f"   Content type: {response.headers.get('content-type', 'unknown')}")
            print(f"   Content length: {len(response.content)}")
    except Exception as e:
        print(f"❌ /video: {e}")
    
    # Try to get livestream frame
    try:
        response = requests.get(f"{base_url}/frame", timeout=5)
        print(f"✅ /frame: {response.status_code}")
        if response.status_code == 200:
            print(f"   Content type: {response.headers.get('content-type', 'unknown')}")
            print(f"   Content length: {len(response.content)}")
            
            # Try to decode as image
            try:
                img_data = response.content
                img_array = np.frombuffer(img_data, dtype=np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if img is not None:
                    print(f"   Image decoded successfully: {img.shape}")
                else:
                    print("   Failed to decode image")
            except Exception as e:
                print(f"   Image decode error: {e}")
    except Exception as e:
        print(f"❌ /frame: {e}")

if __name__ == "__main__":
    test_isaac_livestream_client()
