"""
@brief This file contains the declarations of types of light effects.

@file LightEffect.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

from enum import Enum

class LightEffect(Enum):
    """
    Enum type that defines different robot light effects
    """
    STATIC_RED        = 0   # Static red light.
    STATIC_GREEN      = 1   # Static green light.
    STATIC_BLUE       = 2   # Static blue light.
    STATIC_CYAN       = 3   # Static cyan light.
    STATIC_PURPLE     = 4   # Static purple light.
    STATIC_YELLOW     = 5   # Static yellow light.
    STATIC_WHITE      = 6   # Static white light.
    LOW_FLASH_RED     = 7   # Low-intensity flashing red light (slow bursts).
    LOW_FLASH_GREEN   = 8   # Low-intensity flashing green light (slow bursts).
    LOW_FLASH_BLUE    = 9   # Low-intensity flashing blue light (slow bursts).
    LOW_FLASH_CYAN    = 10  # Low-intensity flashing cyan light (slow bursts).
    LOW_FLASH_PURPLE  = 11  # Low-intensity flashing purple light (slow bursts).
    LOW_FLASH_YELLOW  = 12  # Low-intensity flashing yellow light (slow bursts).
    LOW_FLASH_WHITE   = 13  # Low-intensity flashing white light (slow bursts).
    FAST_FLASH_RED    = 14  # Fast flashing red light (quick bursts).
    FAST_FLASH_GREEN  = 15  # Fast flashing green light (quick bursts).
    FAST_FLASH_BLUE   = 16  # Fast flashing blue light (quick bursts).
    FAST_FLASH_CYAN   = 17  # Fast flashing cyan light (quick bursts).
    FAST_FLASH_PURPLE = 18  # Fast flashing purple light (quick bursts).
    FAST_FLASH_YELLOW = 19  # Fast flashing yellow light (quick bursts).
    FAST_FLASH_WHITE  = 20   # Fast flashing white light (quick bursts).
    