"""
Ground Removal Plugin Package Initialization (core/plugin/ground_removal/__init__.py).
"""

from core.plugin.ground_removal.base_ground_removal import BaseGroundRemoval
from core.plugin.ground_removal.patchworkpp import PatchworkPP

__all__ = ["BaseGroundRemoval", "PatchworkPP"]
