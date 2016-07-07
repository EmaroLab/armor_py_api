"""
Client manager for Armor Python API --ArmorPy.
"""

from armor_msgs.srv import ArmorDirective
import rospy

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"


# =========================================     CLIENT      ========================================= #


name = 'armor_interface_srv'
handle = rospy.ServiceProxy(name, ArmorDirective)
timeout = None


def call(client_id, reference_name, first_dir, second_dir, third_dir, args_list):
    rospy.wait_for_service(name, timeout)
    return handle(client_id, reference_name, first_dir, second_dir, third_dir, args_list)


def set_client_name(new_name):
    global name
    global handle

    name = new_name
    handle = rospy.ServiceProxy(new_name, ArmorDirective)


def set_client_timeout(new_timeout):
    global timeout
    timeout = new_timeout

