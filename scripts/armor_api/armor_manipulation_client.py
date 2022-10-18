#!/usr/bin/env python

"""
Manipulation commands for Armor Python API --ArmorPy.
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


# =========================================  MANIPULATIONS  ========================================= #

class ArmorManipulationClient(object):
    
    _client = None
    
    def __init__(self, client):
        self._client = client
    
    def add_ind_to_class(self, ind_name, class_name):
        """
        Add an individual to a class.
    
        Args:
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
            res = self._client.call('ADD', 'IND', 'CLASS', [ind_name, class_name])
    
        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon adding individual {0} to class {1}: {2}".format(ind_name, class_name, e))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    
    def disj_inds_of_class(self, class_name):
        """
        Disjoint all individuals of a class.

        Args:
        class_name (str): class of the individuals to disjoint.

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
            res = self._client.call('DISJOINT', 'IND', 'CLASS', [class_name])

        except rospy.ServiceException as e:
            raise ArmorServiceCallError("Service call failed upon adding individual {0} to class {1}: {2}".format(ind_name, class_name, e))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
    
    
    
    
    
    def add_objectprop_to_ind(self, objectprop_name, ind_name, value_obj_name):
        """
        Add an object property to an individual. If the object property to be assigned does not exist, it will be created.
    
        Args:
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
            res = self._client.call('ADD', 'OBJECTPROP', 'IND', [objectprop_name, ind_name, value_obj_name])
    
        except rospy.ServiceException:
            err_msg = "Service call failed upon adding property {0} to individual {1}.".format(objectprop_name, ind_name)
            raise ArmorServiceCallError(err_msg)
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR self: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def add_dataprop_to_ind(self, dataprop_name, ind_name, value_type, value):
        """
        Add a data property to an individual. If the data property to be assigned does not exist, it will be created.
    
        Args:
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
            If *value_type* and *value* does not match, *ArmorServiceInternalError* may be risen or you may break your 
            ontology consistency. Consistency can break even if you send a proper request but the ontology is expecting 
            a different value type.
    
        """
        try:
            res = self._client.call('ADD', 'DATAPROP', 'IND', [dataprop_name, ind_name, value_type, value])
    
        except rospy.ServiceException:
            err_msg = "Service call failed upon adding property {0} to individual {1}.".format(dataprop_name, ind_name)
            raise ArmorServiceCallError(err_msg)
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
            
    def add_batch_dataprop_to_ind(self, ind_name, dataprops):
        """
        Add multiple dataprops to a single individual. Properties are passed as list of list, 
        each element of the root list correspond to a property to add.
    
        Args:
            ind_name (str): individual to assign the data property value.
            dataprops: list of [prop_name, value_type, value] objects
    
        Returns:
            bool: True if ontology is consistent and every call succeeds,
                  returns False on the first failed call
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    
        Note:
            It returns the boolean consistency state of the ontology. This value is not updated to the last operation
            if you are working in buffered reasoner or manipulation mode!
    
        Note:
            If *value_type* and *value* does not match, *ArmorServiceInternalError* may be risen or you may break your 
            ontology consistency. Consistency can break even if you send a proper request but the ontology is expecting 
            a different value type.
    
        """
        for prop in dataprops:
          if not self.add_dataprop_to_ind(prop[0], ind_name, prop[1], prop[2]):
            return False
        return True

    def replace_objectprop_b2_ind(self, objectprop_name, ind_name, new_value, old_value):
        """
        Replace the value of an object property belonging to an individual. If the individual or the property instance
        to be replaced do not exist, they will be created.
    
        Args:
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
            *old_value* is necessary since more than one object property instance may be applied to an individual. 
            Only the specified instance will be replaced.
        """
        try:
            res = self._client.call('REPLACE', 'OBJECTPROP', 'IND', [objectprop_name, ind_name, new_value, old_value])
    
        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon adding property {0} to individual {1}.".format(objectprop_name, ind_name))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def replace_dataprop_b2_ind(self, dataprop_name, ind_name, value_type, new_value, old_value):
        """
        Replace the value of a data property belonging to an individual. If the individual or the property instance to be
        replaced do not exist, they will be created.
    
        Args:
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
            *old_value* is necessary since more than one object property instance may be applied to an individual. 
            Only the specified instance will be replaced.
    
        Note:
            If *value_type* and *value* does not match, *ArmorServiceInternalError* may be risen or you may break your 
            ontology consistency. Consistency can break even if you send a proper request but the ontology is expecting 
            a different value type.
        """
        try:
            res = self._client.call('REPLACE', 'DATAPROP', 'IND', 
                                   [dataprop_name, ind_name, value_type, new_value, old_value])
    
        except rospy.ServiceException:
            err_msg = "Service call failed upon adding property {0} to individual {1}.".format(dataprop_name, ind_name)
            raise ArmorServiceCallError(err_msg)
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def replace_one_dataprop_b2_ind(self, dataprop_name, ind_name, value_type, new_value):
        """
        **Use with care:** Utility function that replace the value of the first returned data property value
        associated to an individual without specifying the old value to be replaced. If the individual or the property 
        instance to be replaced do not exist, they will be created. It is supposed to be used with single valued properties.
    
        Args:
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
            If *value_type* and *value* does not match, *ArmorServiceInternalError* may be risen or you may break your 
            ontology consistency. Consistency can break even if you send a proper request but the ontology is expecting 
            a different value type.
        """
        try:
            query = self._client.query.dataprop_b2_ind(dataprop_name, ind_name)
    
            assert len(query) <= 1
            old_value = query[0] if len(query) == 1 else None
    
            if old_value is None:
                res = self.add_dataprop_to_ind(dataprop_name, ind_name, value_type, new_value)
            else:
                res = self.replace_dataprop_b2_ind(
                    dataprop_name, ind_name, value_type, new_value, old_value)
    
        except AssertionError:
            err_msg = "You are trying to replace a single value of {} belonging to {} " \
                      "but multiple values were found. Check your ontology and the " \
                      "way you add data properties too your individuals.".format(dataprop_name, ind_name)
            raise ArmorServiceInternalError(err_msg, "206")
    
        except rospy.ServiceException:
            err_msg = \
                "Failed to replace single value of data property {0} belonging to {1}.".format(dataprop_name, ind_name)
            raise ArmorServiceCallError(err_msg)
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        return res
        
    def replace_one_objectprop_b2_ind(self, objectprop_name, ind_name, new_value):
        """
        **Use with care:** Utility function that replace the value of the first returned object property value
        associated to an individual without specifying the old value to be replaced. If the individual or the property 
        instance to be replaced do not exist, they will be created. It is supposed to be used with single valued properties.
    
        Args:
            object_name (str): name of the object property to assign.
            ind_name (str): individual to assign the object property value.
            new_value (str): value of the object property to be replaced.
    
        Returns:
            bool: True if ontology is consistent, else False.
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
    
        Note:
            This function is meant to be used when only one or less object property instance is expected to be associated
            to that individual. The function check this condition to protect your ontology and raises
            *ArmorServiceInternalError* if it is not satisfied.
    
        Note:
            It returns the boolean consistency state of the ontology. This value is not updated to the last operation
            if you are working in buffered reasoner or manipulation mode!
    
        """
        try:
            query = self._client.query.objectprop_b2_ind(objectprop_name, ind_name)
    
            assert len(query) <= 1
            old_value = query[0] if len(query) == 1 else None
    
            if old_value is None:
                res = self.add_objectprop_to_ind(objectprop_name, ind_name, new_value)
            else:
                res = self.replace_objectprop_b2_ind(
                    objectprop_name, ind_name, new_value, old_value)
    
        except AssertionError:
            err_msg = "You are trying to replace a single value of {} belonging to {} " \
                      "but multiple values were found. Check your ontology and the " \
                      "way you add object properties too your individuals.".format(objectprop_name, ind_name)
            raise ArmorServiceInternalError(err_msg, "206")
    
        except rospy.ServiceException:
            err_msg = \
                "Failed to replace single value of data property {0} belonging to {1}.".format(objectprop_name, ind_name)
            raise ArmorServiceCallError(err_msg)
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        return res

    def remove_dataprop_from_ind(self, dataprop_name, ind_name, value_type, value):
        """
                Remove a data property from an individual.

        Args:
            dataprop_name (str): name of the data property to remove.
            ind_name (str): individual from which to remove the desired data property value.
            value_type (str): type of the value to remove (INTEGER, INT, FLOAT, LONG, DOUBLE, STRING, BOOLEAN, BOOL).
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
            If *value_type* and *value* does not match, *ArmorServiceInternalError* may be risen or you may break your
            ontology consistency. Consistency can break even if you send a proper request but the ontology is expecting
            a different value type.
        """
        try:
            res = self._client.call('REMOVE', 'DATAPROP', 'IND', [dataprop_name, ind_name, value_type, value])

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon removing data property {0} to individual {1}: {2}".format(dataprop_name,ind_name, e))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
            
    def remove_objectprop_from_ind(self, objectprop_name, ind_name, value):
        """
                Remove a object property from an individual.

        Args:
            objectprop_name (str): name of the objet property to remove.
            ind_name (str): individual from which to remove the desired data property value.
            value (str): value to remove as a string.

        Returns:
            bool: True if ontology is consistent, else False

        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

        Note:
            It returns the boolean consistency state of the ontology. This value is not updated to the last operation
            if you are working in buffered reasoner or manipulation mode!

        Note:
            If *value_type* and *value* does not match, *ArmorServiceInternalError* may be risen or you may break your
            ontology consistency. Consistency can break even if you send a proper request but the ontology is expecting
            a different value type.
        """
        try:
            res = self._client.call('REMOVE', 'OBJECTPROP', 'IND', [objectprop_name, ind_name, value])

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon removing object property {0} to individual {1}: {2}".format(objectprop_name, ind_name, e))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def remove_batch_dataprop_to_ind(self, ind_name, dataprops):
        """
        Remove multiple data properties from a single individual. Properties are passed as list of list,
        each element of the root list correspond to a property to remove.

        Args:
            ind_name (str): individual from which to remove the data properties values.
            dataprops: list of [prop_name, value_type, value] objects

        Returns:
            bool: True if ontology is consistent and every call succeeds,
                  returns False on the first failed call

        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error

        Note:
            It returns the boolean consistency state of the ontology. This value is not updated to the last operation
            if you are working in buffered reasoner or manipulation mode!

        Note:
            If *value_type* and *value* does not match, *ArmorServiceInternalError* may be risen or you may break your
            ontology consistency. Consistency can break even if you send a proper request but the ontology is expecting
            a different value type.

        """
        for prop in dataprops:
            if not self.remove_dataprop_from_ind( ind_name,prop[0], prop[1], prop[2]):
                return False
        return True


    def remove_ind_from_class(self, ind_name, class_name):
        """
        Remove an individual from a class.

        Args:
            ind_name (str): individual to be removed from the class.
            class_name (str): individual will be removed to this class. 

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
            res = self._client.call('REMOVE', 'IND','CLASS', [ind_name,class_name])

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon adding individual {0}: {1}".format(ind_name, e))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
