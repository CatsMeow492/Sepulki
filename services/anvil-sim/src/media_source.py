#!/usr/bin/env python3
"""
Media Source for Isaac Sim Video Streaming
Creates a media source that aiortc can reliably consume for WebRTC streaming.
"""

import asyncio
import io
import time
from typing import Optional
import structlog

try:
    import cv2
    import numpy as np
    from aiortc.contrib.media import MediaStreamError
    from aiortc import VideoStreamTrack
    from av import VideoFrame
    MEDIA_AVAILABLE = True
except ImportError:
    MEDIA_AVAILABLE = False
    VideoFrame = None
    # Mock VideoStreamTrack for import
    class VideoStreamTrack:
        def __init__(self): pass

from video_frame_generator import get_video_generator

logger = structlog.get_logger(__name__)

class IsaacSimMediaSource(VideoStreamTrack):
    """
    A media source that generates Isaac Sim video frames compatible with aiortc.
    This follows aiortc's MediaPlayer pattern for reliable video streaming.
    """
    
    def __init__(self, client_id: str, fps: int = 30):
        super().__init__()  # Initialize VideoStreamTrack
        self.client_id = client_id
        self.fps = fps
        self.frame_time = 1.0 / fps
        self.start_time = time.time()
        self.frame_count = 0
        self.running = True
        
        if MEDIA_AVAILABLE:
            self.video_generator = get_video_generator()
            logger.info("🎬 Isaac Sim media source initialized", 
                       client_id=client_id, fps=fps)
        else:
            logger.error("❌ Media dependencies not available")
            
    async def recv(self):
        """
        Generate next video frame for aiortc MediaPlayer consumption.
        This method is called by aiortc's media pipeline.
        """
        if not MEDIA_AVAILABLE:
            raise MediaStreamError("Media dependencies not available")
            
        try:
            # CRITICAL: Use aiortc's proper timestamp mechanism
            pts, time_base = await self.next_timestamp()
            
            logger.info(f"🎬 MEDIA SOURCE RECV() CALLED! Frame #{self.frame_count}", 
                       client_id=self.client_id, pts=pts, time_base=time_base)
            
            # Generate Isaac Sim frame
            frame_data = self.video_generator.generate_frame()
            
            logger.info("✅ Generated Isaac Sim media frame", client_id=self.client_id,
                       frame_shape=frame_data.shape, frame_count=self.frame_count)
            
            # Convert to av.VideoFrame with aiortc timestamp
            frame = VideoFrame.from_ndarray(frame_data, format="bgr24")
            frame.pts = pts
            frame.time_base = time_base
            
            self.frame_count += 1
            
            logger.info("📹 Isaac Sim media frame ready for streaming", 
                       client_id=self.client_id, pts=pts, frame_count=self.frame_count)
            
            return frame
            
        except Exception as e:
            logger.error("❌ Media source frame generation failed", 
                        client_id=self.client_id, error=str(e))
            import traceback
            logger.error("❌ Full traceback", traceback=traceback.format_exc())
            
            # Return bright yellow test frame for debugging
            try:
                pts, time_base = await self.next_timestamp()
                test_frame = np.full((480, 640, 3), [0, 255, 255], dtype=np.uint8)  # Cyan
                frame = VideoFrame.from_ndarray(test_frame, format="bgr24")
                frame.pts = pts
                frame.time_base = time_base
                self.frame_count += 1
                
                logger.warning("⚠️ Returning cyan test frame", client_id=self.client_id)
                return frame
            except Exception as fallback_error:
                logger.error("❌ Even fallback frame failed!", error=str(fallback_error))
                return None
    
    def close(self):
        """Close the media source."""
        self.running = False
        logger.info("🔌 Isaac Sim media source closed", client_id=self.client_id)

# Global media source registry
_media_sources = {}

def create_media_source(client_id: str) -> IsaacSimMediaSource:
    """Create a new media source for a client."""
    source = IsaacSimMediaSource(client_id)
    _media_sources[client_id] = source
    return source

def get_media_source(client_id: str) -> Optional[IsaacSimMediaSource]:
    """Get existing media source for a client."""
    return _media_sources.get(client_id)

def close_media_source(client_id: str):
    """Close and remove media source for a client."""
    source = _media_sources.pop(client_id, None)
    if source:
        source.close()
        logger.info("🗑️ Media source removed", client_id=client_id)
