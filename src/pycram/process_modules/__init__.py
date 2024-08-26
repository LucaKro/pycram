from .pr2_process_modules import Pr2Manager
from .boxy_process_modules import BoxyManager
from .donbot_process_modules import DonbotManager
from .hsrb_process_modules import HSRBManager
from .hsrb_process_modules import HSRBManager
from .armar6_process_modules import ARMAR6Manager
from .stretch_process_modules import StretchManager
from .tiago_process_modules import tiagoManager
from .default_process_modules import DefaultManager


Pr2Manager()
BoxyManager()
DonbotManager()
HSRBManager()
ARMAR6Manager()
tiagoManager()
StretchManager()
DefaultManager()

