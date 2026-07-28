"""
APROS Resource Manager
Offline GUI resources and asset paths manager.
"""
import os
import json

class ResourceManager:
    def __init__(self, resource_dir=None):
        if resource_dir is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            resource_dir = os.path.join(base_dir, "resource")
        self.resource_dir = resource_dir
        self.ensure_resource_dir()
        
    def ensure_resource_dir(self):
        os.makedirs(self.resource_dir, exist_ok=True)
        # Create subfolders for offline resources if needed
        os.makedirs(os.path.join(self.resource_dir, "models"), exist_ok=True)
        os.makedirs(os.path.join(self.resource_dir, "textures"), exist_ok=True)
        os.makedirs(os.path.join(self.resource_dir, "icons"), exist_ok=True)
        
    def get_path(self, relative_path: str) -> str:
        return os.path.join(self.resource_dir, relative_path)
