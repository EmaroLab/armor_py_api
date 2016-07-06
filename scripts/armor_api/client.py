"""
Client manager for Armor Python API --ArmorPy.
"""

from armor_msgs.srv import ArmorDirective
import rospy

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, EmaroLab"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"


# =========================================     CLIENT      ========================================= #


class Client:
    ARMOR_CLIENT_NAME = 'armor_interface_srv'
    ARMOR_CLIENT = rospy.ServiceProxy(ARMOR_CLIENT_NAME, ArmorDirective)
    SERVICE_TIMEOUT = 5
