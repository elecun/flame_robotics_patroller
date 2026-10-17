"""Synerex SYN3100 GNSS receiver integration.

The serial protocol follows the existing Synerex RTK NMEA interface. If a
firmware variant changes its sentence format, parsing remains isolated in the
shared SynerexRTK implementation.
"""
from core.device.synerex_rtk import SynerexRTK, SynerexRTK_Connector


class SynerexSyn3100(SynerexRTK):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault("name", "synerex_syn3100")
        super().__init__(*args, **kwargs)


class SynerexSyn3100_Connector(SynerexRTK_Connector):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault("device_name", "synerex_syn3100")
        super().__init__(*args, **kwargs)
