#!/usr/bin/env python3
"""
Test Real Isaac Sim Integration
Tests the real Isaac Sim renderer and service integration.
"""

import asyncio
import sys
import os

# Add Isaac Sim to Python path
isaac_sim_path = os.path.expanduser("~/isaac-sim/isaac-sim-2023.1.1")
sys.path.insert(0, isaac_sim_path)

# Add service path
sys.path.append(os.path.dirname(__file__))

from isaac_sim_real_renderer import get_isaac_sim_real_renderer, ISAAC_SIM_AVAILABLE
import structlog

logger = structlog.get_logger(__name__)

async def test_real_isaac_sim():
    """Test the real Isaac Sim renderer."""
    print("🧪 Testing Real Isaac Sim Integration...")
    
    if not ISAAC_SIM_AVAILABLE:
        print("❌ Isaac Sim not available")
        return False
    
    try:
        # Get the real Isaac Sim renderer
        renderer = get_isaac_sim_real_renderer()
        print("✅ Real Isaac Sim renderer obtained")
        
        # Test robot loading
        robot_config = {
            'name': 'Test Robot',
            'isaac_sim_path': '/Isaac/Robots/Franka/franka_alt_fingers.usd',
            'specifications': {
                'dof': 7,
                'payload_kg': 3,
                'reach_m': 0.85
            }
        }
        
        success = await renderer.load_robot(robot_config)
        if success:
            print("✅ Robot loaded successfully")
        else:
            print("❌ Robot loading failed")
        
        # Test camera setup
        success = await renderer.setup_camera([4, 4, 4], [0, 0, 0], 50)
        if success:
            print("✅ Camera setup successfully")
        else:
            print("❌ Camera setup failed")
        
        # Test frame rendering
        print("🎬 Testing frame rendering...")
        for i in range(5):
            frame = await renderer.render_frame()
            if frame is not None:
                print(f"✅ Frame {i+1} rendered: {frame.shape}")
            else:
                print(f"❌ Frame {i+1} failed")
        
        # Test joint control
        print("🔧 Testing joint control...")
        joint_states = {'joint1': 0.5, 'joint2': -0.3}
        await renderer.update_joints(joint_states)
        print("✅ Joint control updated")
        
        # Test camera control
        print("📹 Testing camera control...")
        renderer.update_camera([5, 5, 5], [1, 1, 1], 60)
        print("✅ Camera control updated")
        
        # Cleanup
        renderer.cleanup()
        print("✅ Real Isaac Sim renderer cleaned up")
        
        print("🎉 All tests passed!")
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_service_integration():
    """Test the service integration with real Isaac Sim."""
    print("\n🔧 Testing Service Integration...")
    
    try:
        from main import AnvilSimService
        
        # Create service
        service = AnvilSimService()
        print("✅ Service created")
        
        # Test initialization
        await service.initialize_isaac_sim()
        print("✅ Service initialized")
        
        # Test session creation
        session_data = {
            'user_id': 'test_user',
            'sepulka_id': 'test_robot',
            'isaac_sim_robot': {
                'name': 'Test Robot',
                'isaac_sim_path': '/Isaac/Robots/Franka/franka_alt_fingers.usd',
                'specifications': {
                    'dof': 7,
                    'payload_kg': 3,
                    'reach_m': 0.85
                }
            }
        }
        
        # Create mock request
        class MockRequest:
            def __init__(self, data):
                self.data = data
            
            async def json(self):
                return self.data
        
        request = MockRequest(session_data)
        response = await service.create_scene(request)
        
        if response.status == 200:
            print("✅ Session created successfully")
        else:
            print("❌ Session creation failed")
        
        # Cleanup
        await service.shutdown()
        print("✅ Service shutdown")
        
        print("🎉 Service integration test passed!")
        return True
        
    except Exception as e:
        print(f"❌ Service integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    """Run all tests."""
    print("🚀 Starting Real Isaac Sim Integration Tests...")
    
    # Test 1: Real Isaac Sim renderer
    test1_passed = await test_real_isaac_sim()
    
    # Test 2: Service integration
    test2_passed = await test_service_integration()
    
    # Summary
    print("\n📊 Test Summary:")
    print(f"  Real Isaac Sim Renderer: {'✅ PASSED' if test1_passed else '❌ FAILED'}")
    print(f"  Service Integration: {'✅ PASSED' if test2_passed else '❌ FAILED'}")
    
    if test1_passed and test2_passed:
        print("\n🎉 All tests passed! Real Isaac Sim integration is working!")
        return 0
    else:
        print("\n❌ Some tests failed. Please check the errors above.")
        return 1

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
