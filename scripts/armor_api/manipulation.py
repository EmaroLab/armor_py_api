"""
Manipulation commands for Armor Python API --ArmorPy.
"""

import rospy
import client
from .exceptions import ArmorServiceInternalError, ArmorServiceCallError
from .query import query_dataprop_b2_ind

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"


# =========================================  MANIPULATIONS  ========================================= #


def add_ind_to_class(client_id, reference_name, ind_name, class_name):
    """
    Add an individual to a class.

    Args:
        client_id (str):
        reference_name (str):
        ind_name (str): individual to be added to the class.
        class_name (str): individual will be added to this class. It will be created a new class if it does not exist.

    Returns:
        bool: True if ontology is consistent, else False

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!
    """
    try:
        res = client.call(client_id, reference_name, 'ADD', 'IND', 'CLASS', [ind_name, class_name])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding individual %s to class %s: %s" % (ind_name, class_name, e)
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.is_consistent


def add_objectprop_to_ind(client_id, reference_name, objectprop_name, ind_name, value_obj_name):
    """
    Add an object property to an individual. If the object property to be assigned does not exist, it will be created.

    Args:
        client_id (str):
        reference_name (str):
        objectprop_name (str): name of the object property to assign.
        ind_name (str): individual to assign the data property value.
        value_obj_name (str): name of the individual to be used as property value.

    Returns:
        bool: True if ontology is consistent, else False

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!
    """
    try:
        res = client.call(client_id, reference_name, 'ADD', 'OBJECTPROP', 'IND',
                          [objectprop_name, ind_name, value_obj_name])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % (objectprop_name, ind_name)
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.is_consistent


def add_dataprop_to_ind(client_id, reference_name, dataprop_name, ind_name, value_type, value):
    """
    Add a data property to an individual. If the data property to be assigned does not exist, it will be created.

    Args:
        client_id (str):
        reference_name (str):
        dataprop_name (str): name of the data property to assign.
        ind_name (str): individual to assign the data property value.
        value_type (str): type of the value to assign (INTEGER, INT, FLOAT, LONG, DOUBLE, STRING, BOOLEAN, BOOL).
        value (str): value as a string.

    Returns:
        bool: True if ontology is consistent, else False

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        You are free to set any value type but if the type in *value_type* and *value* does not match you may incur in
        *ArmorServiceInternalError* exceptions or break your ontology consistency. Since you are left free to send any
        value to the ontology, you may get inconsistent results even if you submit a proper request but the data
        property in the ontology is expecting a different specific value type.

    """
    try:
        res = client.call(client_id, reference_name, 'ADD', 'DATAPROP', 'IND',
                          [dataprop_name, ind_name, value_type, value])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % (dataprop_name, ind_name)
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.is_consistent


def replace_objectprop_b2_ind(client_id, reference_name, objectprop_name, ind_name, new_value, old_value):
    """
    Replace the value of an object property belonging to an individual. If the individual or the property instance to be
    replaced do not exist, they will be created.

    Args:
        client_id (str):
        reference_name (str):
        objectprop_name (str): name of the object property to replace.
        ind_name (str): individual whose property needs to be replaced.
        new_value (str): new value of the object property.
        old_value (str): value of the object property to be replaced.

    Returns:
        bool: True if ontology is consistent, else False.

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        *old_value* is necessary because more than one object property with different values may be applied to an
        individual. Only the specified instance will be replaced.
    """
    try:
        res = client.call(client_id, reference_name, 'REPLACE', 'OBJECTPROP', 'IND',
                          [objectprop_name, ind_name, new_value, old_value])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % (objectprop_name, ind_name)
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.is_consistent


def replace_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name, value_type, new_value, old_value):
    """
    Replace the value of a data property belonging to an individual. If the individual or the property instance to be
    replaced do not exist, they will be created.

    Args:
        client_id (str):
        reference_name (str):
        dataprop_name (str): name of the data property to assign.
        ind_name (str): individual to assign the data property value.
        value_type (str): type of the value to assign (INTEGER, INT, FLOAT, LONG, DOUBLE, STRING, BOOLEAN, BOOL).
        new_value (str): new value of the data property.
        old_value (str): value of the data property to be replaced.

    Returns:
        bool: True if ontology is consistent, else False.

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        *old_value* is necessary because more than one object property with different values may be applied to an
        individual. Only the specified instance will be replaced.

    Note:
        You are free to set any value type but if the type in *value_type* and *value* does not match you may incur in
        *ArmorServiceInternalError* exceptions or break your ontology consistency. Since you are left free to send any
        value to the ontology, you may get inconsistent results even if you submit a proper request but the data
        property in the ontology is expecting a different specific value type.
    """
    try:
        res = client.call(client_id, reference_name, 'REPLACE', 'DATAPROP', 'IND',
                          [dataprop_name, ind_name, value_type, new_value, old_value])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % (dataprop_name, ind_name)
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.is_consistent


def replace_one_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name, value_type, new_value):
    """
    **Use with care:** Utility function that replace the value of the first returned data property value
    associated to an individual without specifying the old value to be replaced. Read Notes for further information.
    If the individual or the property instance to be replaced do not exist, they will be created.

    Args:
        client_id (str):
        reference_name (str):
        dataprop_name (str): name of the data property to assign.
        ind_name (str): individual to assign the data property value.
        value_type (str): type of the value to assign (INTEGER, INT, FLOAT, LONG, DOUBLE, STRING, BOOLEAN, BOOL).
        new_value (str): value of the data property to be replaced.

    Returns:
        bool: True if ontology is consistent, else False.

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.

    Note:
        This function is meant to be used when only one or less data property instance is expected to be associated
        to that individual. The function check this condition to protect your ontology and raises
        *ArmorServiceInternalError* if it is not satisfied.

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        You are free to set any value type but if the type in *value_type* and *value* does not match you may incur in
        *ArmorServiceInternalError* exceptions or break your ontology consistency. Since you are left free to send any
        value to the ontology, you may get inconsistent results even if you submit a proper request but the data
        property in the ontology is expecting a different specific value type.
    """
    try:
        query = query_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name)

        assert len(query) <= 1
        old_value = query[0] if len(query) == 1 else None

        if old_value is None:
            res = add_dataprop_to_ind(client_id, reference_name, dataprop_name, ind_name, value_type, new_value)
        else:
            res = replace_dataprop_b2_ind(
                client_id, reference_name, dataprop_name, ind_name, value_type, new_value, old_value)

    except AssertionError, e:
        err_msg = "You are trying to replace a single value of %s belonging to %s " \
                  "but multiple values were found. Check your ontology and the " \
                  "way you add data properties too your individuals." % (dataprop_name, ind_name)
        raise ArmorServiceInternalError(err_msg, "206")

    except rospy.ServiceException, e:
        err_msg = "Failed to replace single value of data property %s belonging to %s." % (dataprop_name, ind_name)
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    return res
