from armor_msgs.srv import ArmorDirective
import rospy


class ArmorServiceCallError(Exception):
    def __init__(self, message):
        self.msg = message
        super(ArmorServiceCallError, self).__init__(self.msg)


class ArmorServiceInternalError(Exception):
    def __init__(self, message, exit_code):
        self.msg = message
        self.code = exit_code
        err_msg = "ARMOR internal error %s: %s" % (self.code, self.msg)
        super(ArmorServiceInternalError, self).__init__(err_msg)

# =========================================     CLIENT      ========================================= #


class __Config:
    
    ARMOR_CLIENT_NAME = 'armor_interface_srv'
    ARMOR_CLIENT = rospy.ServiceProxy(ARMOR_CLIENT_NAME, ArmorDirective)
    SERVICE_TIMEOUT = 5
    
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'APPLY', '', '', [])
        rospy.loginfo("%sBuffered manipulations to %s applied.%s",
                      __Config.OKGREEN, reference_name, __Config.ENDC)

    except rospy.ServiceException, e:
        err_msg = "Armor service internal error. Buffered changes have not been applied. Exception: %s" % e
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME, __Config.SERVICE_TIMEOUT)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'REASON', '', '', [])
        rospy.loginfo("%sReasoner synced on %s applied.%s",
                      __Config.OKGREEN, reference_name, __Config.ENDC)
        
    except rospy.ServiceException, e:
        err_msg = "Armor service internal error. Buffered reasoner could not be synced."
        raise ArmorServiceCallError(err_msg)
    
    except rospy.ROSException, e:
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'SAVE', 'INFERENCE', '', [filepath])
        rospy.loginfo("%sReference %s saved to %s.%s",
                      __Config.OKGREEN, reference_name, filepath, __Config.ENDC)

    except rospy.ServiceException, e:
        err_msg = "Armor service internal error. Cannot save reference %s to %s." % \
                  reference_name, filepath
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        if mounted:
            res = __Config.ARMOR_CLIENT(client_id, reference_name, 'LOAD', 'FILE', 'MOUNTED',
                               [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])
        else:
            res = __Config.ARMOR_CLIENT(client_id, reference_name, 'LOAD', 'FILE', '',
                               [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])

    except rospy.ServiceException, e:
        err_msgs = "Service call failed upon reference %s from %s" % (reference_name, client_id)
        raise ArmorServiceCallError(err_msgs)

    except rospy.ROSException, e:
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'MOUNT', '', '', [])
        rospy.loginfo("%s%s mounted on %s.%s", __Config.WARNING, client_id, reference_name, __Config.ENDC)

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon mounting %s on reference %s." % client_id, reference_name
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'UNMOUNT', '', '', [])
        rospy.loginfo("%s%s unmounted from %s.%s", __Config.WARNING, client_id, reference_name, __Config.ENDC)

    except rospy.ServiceException, e:
        err_msg = "Service call failed while unmounting %s from reference %s." % client_id, reference_name, e
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def set_log_to_terminal(switch):
    """
    Set ARMOR to log to terminal.

    Args:
        switch (bool):

    Raises:
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        if switch:
            res = __Config.ARMOR_CLIENT('', '', 'LOG', 'SCREEN', 'ON', [])
        else:
            res = __Config.ARMOR_CLIENT('', '', 'LOG', 'SCREEN', 'OFF', [])
        rospy.loginfo("%sToggled log to terminal.%s", __Config.WARNING, __Config.ENDC)

    except rospy.ServiceException, e:
        err_msg = "Service call failed while toggling logger to terminal"
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        if switch:
            if filepath != '':
                res = __Config.ARMOR_CLIENT('', '', 'LOG', 'FILE', 'ON', [filepath])
            else:
                err_msg = "PYARMOR ERROR: please select a filepath to log to when you toggle on ARMOR logging to file."
                raise ArmorServiceCallError(err_msg)
        else:
            res = __Config.ARMOR_CLIENT('', '', 'LOG', 'FILE', 'OFF', [])

        rospy.loginfo("%sToggled log to file: %s.%s", __Config.WARNING, filepath, __Config.ENDC)

    except rospy.ServiceException, e:
        err_msg = "Service call failed while toggling log to file."
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def set_service_timeout(seconds):
    __Config.SERVICE_TIMEOUT = seconds


def set_service_client(client_name):
    __Config.ARMOR_CLIENT_NAME = client_name
    __Config.ARMOR_CLIENT = rospy.ServiceProxy(__Config.ARMOR_CLIENT_NAME, ArmorDirective)

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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'ADD', 'IND', 'CLASS', [ind_name, class_name])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding individual %s to class %s: %s" % ind_name, class_name
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'ADD', 'OBJECTPROP', 'IND',
                           [objectprop_name, ind_name, value_obj_name])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % objectprop_name, ind_name
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
        ArmorServiceCallError: if call to ARMOR fails
        ArmorServiceInternalError: if ARMOR reports an internal error

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        You are free to set any value type but if the type in *value_type* and *value* does not match you may incur in
        *ArmorServiceInternalError* exception or break your ontology consistency. Since you are left free to send any
        value to the ontology, you may get inconsistent results even if you submit a proper request but the data
        property in the ontology is expecting a different specific value type.

    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'ADD', 'DATAPROP', 'IND',
                           [dataprop_name, ind_name, value_type, value])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % dataprop_name, ind_name
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
        ArmorServiceCallError: if call to ARMOR fails.
        ArmorServiceInternalError: if ARMOR reports an internal error.

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        *old_value* is necessary because more than one object property with different values may be applied to an
        individual. Only the specified instance will be replaced.
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'REPLACE', 'OBJECTPROP', 'IND',
                           [objectprop_name, ind_name, new_value, old_value])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % objectprop_name, ind_name
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
        ArmorServiceCallError: if call to ARMOR fails.
        ArmorServiceInternalError: if ARMOR reports an internal error.

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        *old_value* is necessary because more than one object property with different values may be applied to an
        individual. Only the specified instance will be replaced.

    Note:
        You are free to set any value type but if the type in *value_type* and *value* does not match you may incur in
        *ArmorServiceInternalError* exception or break your ontology consistency. Since you are left free to send any
        value to the ontology, you may get inconsistent results even if you submit a proper request but the data
        property in the ontology is expecting a different specific value type.
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'REPLACE', 'DATAPROP', 'IND',
                           [dataprop_name, ind_name, value_type, new_value, old_value])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon adding property %s to individual %s." % dataprop_name, ind_name
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
        ArmorServiceCallError: if call to ARMOR fails.
        ArmorServiceInternalError: if ARMOR reports an internal error.

    Note:
        This function is meant to be used when only one or less data property instance is expected to be associated
        to that individual. The function check this condition to protect your ontology and raises
        *ArmorServiceInternalError* if it is not satisfied.

    Note:
        It returns the boolean consistency state of the ontology. This value is not updated to the last operation
        if you are working in buffered reasoner or manipulation mode!

    Note:
        You are free to set any value type but if the type in *value_type* and *value* does not match you may incur in
        *ArmorServiceInternalError* exception or break your ontology consistency. Since you are left free to send any
        value to the ontology, you may get inconsistent results even if you submit a proper request but the data
        property in the ontology is expecting a different specific value type.
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
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
                  "way you add data properties too your individuals." % dataprop_name, ind_name
        raise ArmorServiceInternalError(err_msg, "206")

    except rospy.ServiceException, e:
        err_msg = "Failed to replace single value of data property %s belonging to %s." % dataprop_name, ind_name
        raise ArmorServiceCallError(err_msg)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if not res.success:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
    else:
        return res.is_consistent


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
        ArmorServiceCallError: if call to ARMOR fails.
        ArmorServiceInternalError: if ARMOR reports an internal error.
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'QUERY', 'IND', 'CLASS', [class_name])

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
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        res = __Config.ARMOR_CLIENT(client_id, reference_name, 'QUERY', 'DATAPROP', 'IND', [dataprop_name, ind_name])

    except rospy.ServiceException, e:
        err_msg = "Service call failed upon querying property %s to individual %s." % dataprop_name, ind_name
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
        ArmorServiceCallError: if call to ARMOR fails.
        ArmorServiceInternalError: if ARMOR reports an internal error.
    """
    try:
        rospy.wait_for_service(__Config.ARMOR_CLIENT_NAME)
        query = query_ind_b2_class(client_id, reference_name, 'Thing')

    except rospy.ServiceException, e:
        err_msgs = "Service call failed upon querying %s from %s" % (reference_name, client_id)
        raise ArmorServiceCallError(err_msgs)

    except rospy.ROSException, e:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if query:
        return True
    else:
        return False



