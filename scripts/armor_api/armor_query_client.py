#!/usr/bin/env python

"""
Querying commands for Armor Python API --ArmorPy.
"""

import rospy
from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"

# =========================================      QUERY      ========================================= #


class ArmorQueryClient(object):
    _client = None

    def __init__(self, client):
        self._client = client

    def ind_b2_class(self, class_name):
        """
        Query the list of all individuals belonging to a class.
    
        Args:
            class_name (str): a class in the ontology
    
        Returns:
            list(str): the list of individual belonging to the class
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
        """
        try:
            res = self._client.call('QUERY', 'IND', 'CLASS', [class_name])
    
        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon querying individuals belonging to class {0}".format(class_name))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.queried_objects
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def dataprop_b2_ind(self, dataprop_name, ind_name):
        """
        Query all values of a data property associated with an individual.
    
        Args:
            dataprop_name (str): data property whose values you want to query.
            ind_name (str): individual whose value you want to query.
    
        Returns:
            list(str): list of queried values as strings.
        """
        try:
            res = self._client.call('QUERY', 'DATAPROP', 'IND', [dataprop_name, ind_name])
    
        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon querying property {0} to individual {1}.".format(dataprop_name, ind_name))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.queried_objects
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
            
    def objectprop_b2_ind(self, objectprop_name, ind_name):
        """
        Query all object values of an object property associated with an individual.
    
        Args:
            objectprop_name (str): object property whose values you want to query.
            ind_name (str): individual whose value you want to query.
    
        Returns:
            list(str): list of queried values as strings.
        """
        try:
            res = self._client.call('QUERY', 'OBJECTPROP', 'IND', [objectprop_name, ind_name])
    
        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon querying property {0} to individual {1}.".format(objectprop_name, ind_name))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.queried_objects
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def check_ind_exists(self, ind_name):
        """
        Utility function to check if an arbitrary named individual exists.
    
        Args:
            ind_name (str): the individual to be checked
    
        Returns:
            bool: True if individual exists, else False
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
        """
        try:
            res = self._client.call('QUERY', 'IND', '', [ind_name])
    
        except rospy.ServiceException:
            raise ArmorServiceCallError("Service call failed upon querying {0} from {1}".format(
                self._client.reference_name, self._client.client_id))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success and len(res.queried_objects) > 1:
            return True
        else:
            return False
    
