# -*- coding: utf-8 -*-
"""Reusable BRUNNER controller entrypoint for FSDC QTG tests.

Use ClimbBrunnerApp as the base for new test apps, including Engine:

    from climb_brunner import ClimbBrunnerApp
    class EngineApp(ClimbBrunnerApp):
        ...

Why the full logger dependency?
The climb replay is a coordinated state machine (manual reference + P3D
initialization + BRUNNER AP/force + t=0 physical verification + direct physical
feedback).  It cannot safely be collapsed into just UDP commands.  This base
class reuses the EXACT proven climb methods, while the separate transport
holds the original protocol implementation.  Do not bypass start_auto_test()
with direct AP writes, or remove preflight interlocks to make a test run.

A completely GUI-independent high-level replay controller is NOT claimed here.
"""
from Super_Mushak_Logger_V15_FULL_CAUSAL_STATE import SuperMushakLoggerApp
from climb_brunner_transport import (
    Cls2SimUdpTransport, packet_override, packet_setting_u8, packet_setting_u16,
    packet_setting_i32, packet_setting_float, packet_read, packet_watchdog,
    packet_state, node_id_mask, packet_remote_config_u8,
    AX_ELEVATOR, AX_AILERON, AX_RUDDER, AX_THREE_ACTUATORS,
    SID_AP_POSITION, SID_AP_ENABLE, SID_AP_FORCE, SID_AP_SPEED,
    DATA_FORCE_ENG, DATA_VIRTUAL_FORCE_ENG, DATA_POS_NORM,
)

class ClimbBrunnerApp(SuperMushakLoggerApp):
    """Full climb BRUNNER recording/replay inherited without algorithm edits.

    The base app supplies strict read-only Manual, all-axes Automatic,
    physical t=0 position/force gates, position lookahead, pitch/roll/yaw
    force tracking, P3D state synchronization, safety, telemetry and reports.

    Override test-specific initial conditions, duration, throttle sequence,
    results and reporting in subclasses, NOT low-level BRUNNER controls.
    """
    pass

__all__ = [
    'ClimbBrunnerApp', 'Cls2SimUdpTransport',
    'packet_override', 'packet_setting_u8', 'packet_setting_u16',
    'packet_setting_i32', 'packet_setting_float', 'packet_read',
    'packet_watchdog', 'packet_state', 'node_id_mask',
    'packet_remote_config_u8',
    'AX_ELEVATOR', 'AX_AILERON', 'AX_RUDDER', 'AX_THREE_ACTUATORS',
    'SID_AP_POSITION', 'SID_AP_ENABLE', 'SID_AP_FORCE', 'SID_AP_SPEED',
    'DATA_POS_NORM', 'DATA_FORCE_ENG', 'DATA_VIRTUAL_FORCE_ENG',
]
