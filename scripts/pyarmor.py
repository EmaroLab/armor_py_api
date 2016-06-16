from armor_msgs.srv import ArmorDirective
import rospy


class __TerminalColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


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

armor_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)

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
        It returns the boolean consistency state of the ontology. This value is not updated if you are working in
        buffered reasoner mode!
    """
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'APPLY', '', '', [])
        rospy.loginfo("%sBuffered manipulations to %s applied.%s",
                      __TerminalColors.OKGREEN, reference_name, __TerminalColors.ENDC)
    except rospy.ServiceException, e:
        err_msg = "Armor service internal error. Buffered changes have not been applied. Exception: %s" % e
        rospy.logerr(err_msg)
        raise ArmorServiceCallError(err_msg)
    if res.success:
        return res.is_consistent
    else:
        rospy.logwarn(res.error_description)
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
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'REASON', '', '', [])
        rospy.loginfo("%sReasoner synced on %s applied.%s",
                      __TerminalColors.OKGREEN, reference_name, __TerminalColors.ENDC)
    except rospy.ServiceException, e:
        err_msg = "Armor service internal error. Buffered reasoner could not be synced. Exception: %s" % e
        rospy.logerr(err_msg)
        raise ArmorServiceCallError(err_msg)
    if res.success:
        return res.is_consistent
    else:
        rospy.logwarn(res.error_description)
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
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'SAVE', 'INFERENCE', '', [filepath])
        rospy.loginfo("%sReference %s saved to %s.%s",
                      __TerminalColors.OKGREEN, reference_name, filepath, __TerminalColors.ENDC)
    except rospy.ServiceException, e:
        err_msg = "Armor service internal error. Cannot save reference %s to %s." % \
                  reference_name, filepath
        rospy.logerr(err_msg)
        raise ArmorServiceCallError(err_msg)
    if not res.success:
        rospy.logwarn(res.error_description)
        raise ArmorServiceInternalError(res.error_description, res.exit_code)


def load_ref_from_file(client_id, reference_name, owl_file_path, iri,
                       buffered_manipulation=False, reasoner="PELLET", buffered_reasoner=False, mounted=False):
    rospy.wait_for_service('armor_interface_srv')
    try:
        if mounted:
            res = armor_call(client_id, reference_name, 'LOAD', 'FILE', 'MOUNTED',
                             [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])
            if res.success is False:
                rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
        else:
            res = armor_call(client_id, reference_name, 'LOAD', 'FILE', '',
                             [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])

            if res.success is False:
                rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
    except rospy.ServiceException, e:
        err_msgs = "Service call failed upon reference %s from %s" % (reference_name, client_id)
        rospy.logwarn(err_msgs)
        raise ArmorServiceCallError(err_msgs, e)


def mount_on_ref(client_id, reference_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'MOUNT', '', '', [])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
            return False
        else:
            rospy.loginfo("%s%s mounted on %s.%s", __TerminalColors.WARNING, client_id, reference_name, __TerminalColors.ENDC)
            return True
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon mounting %s on reference %s: %s", client_id, reference_name, e)
        return None


def unmount_from_ref(client_id, reference_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'UNMOUNT', '', '', [])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
            return False
        else:
            rospy.loginfo("%s%s unmounted from %s.%s", __TerminalColors.WARNING, client_id, reference_name, __TerminalColors.ENDC)
            return True
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed while unmounting %s from reference %s: %s", client_id, reference_name, e)
        return None


def set_log_to_terminal(switch):
    rospy.wait_for_service('armor_interface_srv')
    try:
        if switch:
            res = armor_call('', '', 'LOG', 'SCREEN', 'ON', [])
        else:
            res = armor_call('', '', 'LOG', 'SCREEN', 'OFF', [])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
            return False
        else:
            rospy.loginfo("%sToggled log to terminal.%s", __TerminalColors.WARNING, __TerminalColors.ENDC)
            return True
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed while toggling logger to terminal: %s", e)
        return None


def set_log_to_file(switch, filepath=''):
    rospy.wait_for_service('armor_interface_srv')
    try:
        if switch:
            if filepath != '':
                res = armor_call('', '', 'LOG', 'FILE', 'ON', [filepath])
            else:
                rospy.logerr("PYARMOR ERROR: "
                             "please select a filepath to log to when you toggle on ARMOR logging to file.")
                return None
        else:
            res = armor_call('', '', 'LOG', 'FILE', 'OFF', [])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
            return False
        else:
            rospy.loginfo("%sToggled log to file: %s.%s", __TerminalColors.WARNING, filepath, __TerminalColors.ENDC)
            return True
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed while toggling log to file: %s", e)
        return None

# =========================================  MANIPULATIONS  ========================================= #


def add_ind_to_class(client_id, reference_name, ind_name, class_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'ADD', 'IND', 'CLASS', [ind_name, class_name])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding individual %s to class %s: %s", ind_name, class_name, e)
        return None


def add_objectprop_to_ind(client_id, reference_name, objectprop_name, ind_name, value_obj_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'ADD', 'OBJECTPROP', 'IND',
                         [objectprop_name, ind_name, value_obj_name])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding property %s to individual %s: %s", objectprop_name, ind_name, e)
        return None


def add_dataprop_to_ind(client_id, reference_name, dataprop_name, ind_name, value_type, value):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'ADD', 'DATAPROP', 'IND',
                       [dataprop_name, ind_name, value_type, value])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding property %s to individual %s: %s", dataprop_name, ind_name, e)
        return None


def replace_objectprop_to_ind(client_id, reference_name, objectprop_name, ind_name, new_value, old_value):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'REPLACE', 'OBJECTPROP', 'IND',
                           [objectprop_name, ind_name, new_value, old_value])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding property %s to individual %s: %s", objectprop_name, ind_name, e)
        return None


def replace_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name, value_type, new_value, old_value):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'REPLACE', 'DATAPROP', 'IND',
                           [dataprop_name, ind_name, value_type, new_value, old_value])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding property %s to individual %s: %s", dataprop_name, ind_name, e)
        return None


def replace_one_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name, value_type, new_value):
    """
    Use with care! This is an utility function that replace the value of the first returned data property value
    associated to an individual without specifying the old value to be replaced. It is supposed to be used only when
    it is guaranteed that one or less data property value associated to that individual. Else, it will fail.
    :param client_id:
    :param reference_name:
    :param dataprop_name:
    :param ind_name:
    :param value_type:
    :param new_value:
    :return:
    """
    rospy.wait_for_service('armor_interface_srv')
    query = query_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name)
    if query is not None:
        try:
            assert len(query.queried_objects) <= 1
            old_value = query.queried_objects[0] if len(query.queried_objects) == 1 else None
            if old_value is None:
                res = add_dataprop_to_ind(client_id, reference_name, dataprop_name, ind_name, value_type, new_value)
            else:
                res = replace_dataprop_b2_ind(
                    client_id, reference_name, dataprop_name, ind_name, value_type, new_value, old_value)
            return res
        except AssertionError, e:
            rospy.logerr("%s: You are trying to replace a single value of %s belonging to %s"
                         " but multiple values were found. Check your ontology and the "
                         "way you add data properties too your individuals.", e, dataprop_name, ind_name)
            return None
    else:
        return None


# =========================================      QUERY      ========================================= #

def query_ind_b2_class(client_id, reference_name, ind_name, class_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'QUERY', 'IND', 'CLASS', [ind_name, class_name])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon querying individual %s belonging to class %s: %s",
                      ind_name, class_name, e)
        return None


def query_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        res = armor_call(client_id, reference_name, 'QUERY', 'DATAPROP', 'IND', [dataprop_name, ind_name])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon querying property %s to individual %s: %s", dataprop_name, ind_name, e)
        return None


def check_ind_exists(client_id, reference_name, ind_name):
    """
    Utility function to check an arbitrary named individual exists.
    WARNING: Returns the first individual returned by the query (if more than one is found)
    Else, it return None
    :param client_id:
    :param reference_name:
    :param ind_name:
    :return:
    """
    rospy.wait_for_service('armor_interface_srv')
    try:
        query = query_ind_b2_class(client_id, reference_name, ind_name, 'Thing')
        return query.queried_objects[0]
    except rospy.ServiceException, e:
        err_msgs = "Service call failed upon querying %s from %s" % (reference_name, client_id)
        rospy.logwarn(err_msgs)
        raise ArmorServiceCallError(err_msgs)


