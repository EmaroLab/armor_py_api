"""
Querying commands for Armor Python API --ArmorPy.
"""

from .exceptions import ArmorServiceInternalError, ArmorServiceCallError
import client
import rospy

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"

# =========================================      QUERY      ========================================= #


def query_ind_b2_class(client_id, reference_name, class_name):
    """
    Query the list of all individuals belonging to a class.

    Args:
        client_id (str):
        reference_name (str):
        class_name (str): a class in the ontology

    Returns:
        list(str): the list of individual belonging to the class

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
    """
    try:
        res = client.call(client_id, reference_name, 'QUERY', 'IND', 'CLASS', [class_name])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon querying individuals belonging to class %s" % class_name
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.queried_objects


def query_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name):
    """
    Query all values of a data property associated with an individual.

    Args:
        client_id (str):
        reference_name (str):
        dataprop_name (str): data property whose value you want to query.
        ind_name (str): individual whose value you want to query.

    Returns:
        list(str): list of queried values as strings.
    """
    try:
        res = client.call(client_id, reference_name, 'QUERY', 'DATAPROP', 'IND', [dataprop_name, ind_name])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon querying property %s to individual %s." % (dataprop_name, ind_name)
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.queried_objects


def check_ind_exists(client_id, reference_name, ind_name):
    """
    Utility function to check if an arbitrary named individual exists.

    Args:
        client_id (str):
        reference_name (str):
        ind_name (str): the individual to be checked

    Returns:
        bool: True if individual exists, else False

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
    """
    try:
        query = query_ind_b2_class(client_id, reference_name, 'Thing')

    except rospy.ServiceException, e:
        err_msgs = "Service call failed upon querying %s from %s" % (reference_name, client_id)
        raise ArmorServiceCallError(err_msgs)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if ind_name in query:
        return True
    else:
        return False

