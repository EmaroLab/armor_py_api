#!/usr/bin/env python

"""
Python client manager class for Armor --ArmorPy.
"""

from armor_msgs.srv import ArmorDirective, ArmorDirectiveList, ArmorDirectiveListRequest
from armor_msgs.msg import _ArmorDirectiveReq
from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError
from armor_api.armor_manipulation_client import ArmorManipulationClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient
import rospy

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"


# =========================================     CLIENT      ========================================= #


class ArmorClient(object):
    _handle = None
    _serial_handle = None
    _service_name = None
    _serial_service_name = None
    _buffered_commands_list = None

    reference_name = None
    client_id = None
    timeout = 0

    manipulation = None
    query = None
    utils = None

    # TODO add support for list of queries

    def __init__(self, client_id, reference_name, service_name='/armor_interface_srv',
                 serial_service_name='/armor_interface_serialized_srv', timeout=5):
        self.reference_name = reference_name
        self.client_id = client_id
        self.timeout = timeout
        self._service_name = service_name
        self._serial_service_name = serial_service_name
        self._handle = rospy.ServiceProxy(self._service_name, ArmorDirective)
        self._serial_handle = rospy.ServiceProxy(self._serial_service_name, ArmorDirectiveList)

        self.manipulation = ArmorManipulationClient(self)
        self.query = ArmorQueryClient(self)
        self.utils = ArmorUtilsClient(self)

    def call(self, command, first_spec, second_spec, args_list):
        req = self._prepare_request(command, first_spec, second_spec, args_list)
        rospy.wait_for_service(self._service_name, self.timeout)
        res = self._handle(req).armor_response
        return res

    def _prepare_request(self, command, first_spec, second_spec, args_list):
        req = _ArmorDirectiveReq.ArmorDirectiveReq()
        req.client_name = self.client_id
        req.reference_name = self.reference_name
        req.command = command
        req.primary_command_spec = first_spec
        req.secondary_command_spec = second_spec
        req.args = args_list
        return req
