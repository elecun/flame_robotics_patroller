"""Compatibility-facing module for the Synerex SYN2000 RTK receiver."""
from core.device.synerex_rtk import SynerexRTK, SynerexRTK_Connector


class SynerexSyn2000(SynerexRTK):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault("name", "synerex_syn2000")
        super().__init__(*args, **kwargs)


class SynerexSyn2000_Connector(SynerexRTK_Connector):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault("device_name", "synerex_syn2000")
        super().__init__(*args, **kwargs)
