"""roborun-link: the session client, observation capture/encode/upload and the pack cache.

Normal priority, separate process from roborun-rt. It may crash or reconnect without touching roborun-rt; a
reconnect never resumes motion (rt latched the link loss and set a scene-change mark; the next motion starts from the
measured rest state). It holds no motor writes. JPEG/zstd encoding lives here, never in rt.
"""
