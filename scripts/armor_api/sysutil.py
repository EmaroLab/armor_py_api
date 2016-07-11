"""
System utility commands for Armor Python API --ArmorPy.
"""

import rospy
import client
from .exceptions import ArmorServiceCallError, ArmorServiceInternalError, __TermColors

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"

# =========================================     UTILITY     ========================================= #


def apply_buffered_changes(client_id, reference_name):
    """
    Apply buffered manipulations to the ontology when its reference has been initialized in buffered manipulation mode

    Args:
        client_id (str):
        reference_name (str):

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
        res = client.call(client_id, reference_name, 'APPLY', '', '', [])
        rospy.loginfo(
            "{0}Buffered manipulations to {1} applied.{2}"
            .format(__TermColors.OKGREEN, reference_name, __TermColors.ENDC))

    except rospy.ServiceException, e:
        raise ArmorServiceCallError(
            "Armor service internal error. Buffered changes have not been applied. Exception: {0}".format(e))

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if res.success:
        return res.is_consistent
    else:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def sync_buffered_reasoner(client_id, reference_name):
    """
    Sync ontology reference reasoner when operating in buffered reasoner mode

    Args:
        client_id (str):
        reference_name (str):

    Returns:
        bool: True if ontology is consistent, False otherwise

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        res = client.call(client_id, reference_name, 'REASON', '', '', [])
        rospy.loginfo(
            "{0}Reasoner synced on {1} applied.{2}".format(__TermColors.OKGREEN, reference_name, __TermColors.ENDC))
        
    except rospy.ServiceException:
        raise ArmorServiceCallError("Armor service internal error. Buffered reasoner could not be synced.")
    
    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
    if res.success:
        return res.is_consistent
    else:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def save_ref_with_inferences(client_id, reference_name, filepath):
    """
    Save ontology with latest inferred axioms to specified filepath

    Args:
        client_id (str):
        reference_name (str):
        filepath (str):

    Returns:
        None:

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        res = client.call(client_id, reference_name, 'SAVE', 'INFERENCE', '', [filepath])
        rospy.loginfo(
            "{0}Reference {1} saved to {2}.{3}"
            .format(__TermColors.OKGREEN, reference_name, filepath, __TermColors.ENDC))

    except rospy.ServiceException:
        raise ArmorServiceCallError(
            "Armor service internal error. Cannot save reference {0} to {1}.".format(reference_name, filepath))

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def load_ref_from_file(client_id, reference_name, owl_file_path, iri,
                       buffered_manipulation=False, reasoner="PELLET", buffered_reasoner=False, mounted=False):
    """
    Loads an ontology into armor from an .owl file.

    Args:
        client_id (str):
        reference_name (str):
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
            res = client.call(client_id, reference_name, 'LOAD', 'FILE', 'MOUNTED',
                              [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])
        else:
            res = client.call(client_id, reference_name, 'LOAD', 'FILE', '',
                              [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])

    except rospy.ServiceException:
        raise ArmorServiceCallError(
            "Service call failed upon reference {0} from {1}".format(reference_name, client_id))

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def mount_on_ref(client_id, reference_name):
    """
    Mounts a client on an ontology reference.

    Args:
        client_id (str):
        reference_name (str):

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        res = client.call(client_id, reference_name, 'MOUNT', '', '', [])
        rospy.loginfo(
            "{0}{1} mounted on {2}.{3}".format(__TermColors.WARNING, client_id, reference_name, __TermColors.ENDC))

    except rospy.ServiceException:
        raise ArmorServiceCallError(
            "Service call failed upon mounting {0} on reference {1}.".format(client_id, reference_name))

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def unmount_from_ref(client_id, reference_name):
    """
    Unmount a client from an ontology.

    Args:
        client_id (str):
        reference_name (str):

    Raises:
        armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
        armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        res = client.call(client_id, reference_name, 'UNMOUNT', '', '', [])
        rospy.loginfo(
            "{0}{1} unmounted from {2}.{3}".format(__TermColors.WARNING, client_id, reference_name, __TermColors.ENDC))

    except rospy.ServiceException:
        raise ArmorServiceCallError(
            "Service call failed while unmounting {0} from reference {1}.".format(client_id, reference_name))

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def set_log_to_terminal(switch):
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
            res = client.call('', '', 'LOG', 'SCREEN', 'ON', [])
        else:
            res = client.call('', '', 'LOG', 'SCREEN', 'OFF', [])
        rospy.loginfo("{0}Toggled log to terminal.{1}".format(__TermColors.WARNING, __TermColors.ENDC))

    except rospy.ServiceException:
        raise ArmorServiceCallError("Service call failed while toggling logger to terminal")

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def set_log_to_file(switch, filepath=''):
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
                res = client.call('', '', 'LOG', 'FILE', 'ON', [filepath])
            else:
                raise ArmorServiceCallError(
                    "ARMOR_PY ERROR: please select a filepath to log to when you toggle on ARMOR logging to file.")
        else:
            res = client.call('', '', 'LOG', 'FILE', 'OFF', [])

        rospy.loginfo("{0}Toggled log to file: {1}.{2}".format(__TermColors.WARNING, filepath, __TermColors.ENDC))

    except rospy.ServiceException:
        raise ArmorServiceCallError("Service call failed while toggling log to file.")

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
