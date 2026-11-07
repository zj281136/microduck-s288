"""Minimal registry for the published S288 Hybrid identification models.

Packaging-only subset; no other actuator families are registered.
"""
actuators = {}
from .unitree.identification12.model import register
register()
