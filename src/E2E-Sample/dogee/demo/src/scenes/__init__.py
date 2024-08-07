#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from sys import modules

from src.scenes.lane_following import LF
from src.scenes.manual import Manual
from src.scenes.tracking import Tracking
from src.scenes.video_test import VideoTest
from src.utils import log

__all__ = ['Manual', 'Tracking', 'LF', 'scene_initiator']


def scene_initiator(name):
    """
    Get the object of the scene by name.
    @param name: the name of the scene
    @return: the object of the scene
    """
    try:
        scene = getattr(modules.get(__name__), name)
    except AttributeError:
        log.error(f"{name} doesn't exist.")
        return None
    if isinstance(scene, type):
        return scene

    log.error(f"{name} is not a valid scene.")
    return None
