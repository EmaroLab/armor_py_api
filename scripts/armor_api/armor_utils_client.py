#!/usr/bin/env python

"""
System utility commands for Armor Python API --ArmorPy.
"""

import rospy
from armor_api.armor_exceptions import ArmorServiceCallError, ArmorServiceInternalError

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"

# =========================================     UTILITY     ========================================= #


class ArmorUtilsClient(object):
    _client = None

    class __TermColors:
        HEADER = '\033[95m'
        OKBLUE = '\033[94m'
        OKGREEN = '\033[92m'
        WARNING = '\033[93m'
        FAIL = '\033[91m'
        ENDC = '\033[0m'
        BOLD = '\033[1m'
        UNDERLINE = '\033[4m'

    def __init__(self, client):
        self._client = client

    def apply_buffered_changes(self):
        """
        Apply buffered manipulations to the ontology when its reference has been initialized in buffered manipulation mode
    
        Returns:
            bool: True if ontology is consistent, False otherwise
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    
        Note:
            It returns the boolean consistency state of the ontology. This value is not updated to the last operation
            if you are working in buffered reasoner or manipulation mode!
        """
        try:
            res = self._client.call('APPLY', '', '', [])
            rospy.loginfo(
                "{0}Buffered manipulations to {1} applied.{2}"
                .format(self.__TermColors.OKGREEN, self._client.reference_name, self.__TermColors.ENDC))

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Armor service internal error. Buffered changes have not been applied. Exception: {0}".format(e))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def sync_buffered_reasoner(self):
        """
        Sync ontology reference reasoner when operating in buffered reasoner mode
    
        Returns:
            bool: True if ontology is consistent, False otherwise
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            res = self._client.call('REASON', '', '', [])
            rospy.loginfo(
                "{0}Reasoner synced on {1} applied.{2}".format(
                    self.__TermColors.OKGREEN, self._client.reference_name, self.__TermColors.ENDC))

        except rospy.ServiceException:
            raise ArmorServiceCallError("Armor service internal error. Buffered reasoner could not be synced.")

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def save_ref_with_inferences(self, filepath):
        """
        Save ontology with latest inferred axioms to specified filepath
    
        Args:
            filepath (str):
    
        Returns:
            None:
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        self.save(filepath, True)
            
    def save(self, filepath, inference=False):
        """
        Save ontology
    
        Args:
            filepath (str):
    
        Returns:
            None:
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            if inference:
                res = self._client.call('SAVE', 'INFERENCE', '', [filepath])            
            else:
                res = self._client.call('SAVE', '', '', [filepath])
            rospy.loginfo(
                "{0}Reference {1} saved to {2}.{3}"
                .format(self.__TermColors.OKGREEN, self._client.reference_name, filepath, self.__TermColors.ENDC))

        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Armor service internal error. Cannot save reference {0} to {1}.".format(
                    self._client.reference_name, filepath))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)


    def load_ref_from_file(self, owl_file_path, iri,
                           buffered_manipulation=False, reasoner="PELLET", buffered_reasoner=False, mounted=False):
        """
        Loads an ontology into armor from an .owl file.
    
        Args:
            owl_file_path (str):
            iri (str):
            buffered_manipulation (bool): set buffered manipulations, default False
            reasoner (str): set which reasoner to use (PELLET, HERMIT, SNOROCKET, FACT), default PELLET
            buffered_reasoner (bool): set if reasoner should be buffered, default False
            mounted (bool): set if the client should be mounted on the reference, default False
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            if mounted:
                res = self._client.call('LOAD', 'FILE', 'MOUNTED',
                                        [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])
            else:
                res = self._client.call('LOAD', 'FILE', '',
                                        [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])

        except rospy.ServiceException:
            raise ArmorServiceCallError("Service call failed upon reference {0} from {1}"
                                        .format(self._client.reference_name, self._client.client_id))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def mount_on_ref(self):
        """
        Mounts a client on an ontology reference.
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            res = self._client.call('MOUNT', '', '', [])
            rospy.loginfo(
                "{0}{1} mounted on {2}.{3}".format(self.__TermColors.WARNING, self._client.client_id,
                                                   self._client.reference_name, self.__TermColors.ENDC))

        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon mounting {0} on reference {1}.".format(
                    self._client.client_id, self._client.reference_name))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def unmount_from_ref(self):
        """
        Unmount a client from an ontology.
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            res = self._client.call('UNMOUNT', '', '', [])
            rospy.loginfo(
                "{0}{1} unmounted from {2}.{3}".format(self.__TermColors.WARNING, self._client.client_id,
                                                       self._client.reference_name, self.__TermColors.ENDC))

        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed while unmounting {0} from reference {1}.".format(self._client.client_id,
                                                                                      self._client.reference_name))

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def set_log_to_terminal(self, switch):
        """
        Set ARMOR to log to terminal.
    
        Args:
            switch (bool):
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            if switch:
                res = self._client.call('LOG', 'SCREEN', 'ON', [])
            else:
                res = self._client.call('LOG', 'SCREEN', 'OFF', [])
            rospy.loginfo("{0}Toggled log to terminal.{1}".format(self.__TermColors.WARNING, self.__TermColors.ENDC))

        except rospy.ServiceException:
            raise ArmorServiceCallError("Service call failed while toggling logger to terminal")

        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)

    def set_log_to_file(self, switch, filepath=''):
        """
        Set ARMOR to log to file.
    
        Args:
            switch (bool):
            filepath (str):
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
        """
        try:
            if switch:
                if filepath != '':
                    res = self._client.call('LOG', 'FILE', 'ON', [filepath])
                else:
                    raise ArmorServiceCallError(
                        "ARMOR_PY ERROR: please select a filepath to log to when you toggle on ARMOR logging to file.")
            else:
                res = self._client.call('', '', 'LOG', 'FILE', 'OFF', [])
    
            rospy.loginfo("{0}Toggled log to file: {1}.{2}".format(self.__TermColors.WARNING,
                                                                   filepath, self.__TermColors.ENDC))
    
        except rospy.ServiceException:
            raise ArmorServiceCallError("Service call failed while toggling log to file.")
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if not res.success:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
