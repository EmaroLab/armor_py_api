from armor_msgs.srv import ArmorDirective
import rospy

class TerminalColors:
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
    try:
        apply_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = apply_call(client_id, reference_name, 'APPLY', '', '', [])
        rospy.loginfo("%sBuffered manipulations to %s applied.%s",
                      TerminalColors.OKGREEN, reference_name, TerminalColors.ENDC)
        return res.success and res.is_consistent
    except rospy.ServiceException, e:
        rospy.logerr("Armor service internal error. Buffered changes have not been applied. Exception: %s", e)
        return False


def sync_buffered_reasoner(client_id, reference_name):
    try:
        sync_call = rospy.ServiceProxy('armore_interface_srv', ArmorDirective)
        res = sync_call(client_id, reference_name, 'REASON', '', '', [])
        rospy.loginfo("%sReasoner synced on %s applied.%s",
                      TerminalColors.OKGREEN, reference_name, TerminalColors.ENDC)
        return res.success and res.is_consistent
    except rospy.ServiceException, e:
        rospy.logerr("Armor service internal error. Buffered reasoner could not be synced. Exception: %s", e)
        return False


def save_ref_with_inferences(client_id, reference_name, filepath):
    try:
        save_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = save_call(client_id, reference_name, 'SAVE', 'INFERENCE', '', [filepath])
        rospy.loginfo("%sReference %s saved to %s.%s",
                      TerminalColors.OKGREEN, reference_name, filepath, TerminalColors.ENDC)
        return res.success
    except rospy.ServiceException, e:
        rospy.logerr("Armor service internal error. Cannot save reference %s to %s. Exception: %s",
                     reference_name, filepath, e)
        return False


def load_ref_from_file(client_id, reference_name, owl_file_path, iri,
                       buffered_manipulation=False, reasoner="PELLET", buffered_reasoner=False, mounted=False):
    rospy.wait_for_service('armor_interface_srv')
    try:
        load_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        if mounted:
            res = load_call(client_id, reference_name, 'LOAD', 'FILE', 'MOUNTED',
                            [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])
            if res.success is False:
                rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
        else:
            res = load_call(client_id, reference_name, 'LOAD', 'FILE', '',
                            [owl_file_path, iri, str(buffered_manipulation), reasoner, str(buffered_reasoner)])
            if res.success is False:
                rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon creating reference %s: %s", reference_name, e)
        return None


def mount_on_ref(client_id, reference_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        mount_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = mount_call(client_id, reference_name, 'MOUNT', '', '', [])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
            return False
        else:
            rospy.loginfo("%s%s mounted on %s.%s", TerminalColors.WARNING, client_id, reference_name, TerminalColors.ENDC)
            return True
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon mounting %s on reference %s: %s", client_id, reference_name, e)
        return None


def unmount_from_ref(client_id, reference_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        mount_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = mount_call(client_id, reference_name, 'UNMOUNT', '', '', [])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
            return False
        else:
            rospy.loginfo("%s%s unmounted from %s.%s", TerminalColors.WARNING, client_id, reference_name, TerminalColors.ENDC)
            return True
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed while unmounting %s from reference %s: %s", client_id, reference_name, e)
        return None

# =========================================  MANIPULATIONS  ========================================= #


def add_ind_to_class(client_id, reference_name, ind_name, class_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        update_shape_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = update_shape_call(client_id, reference_name, 'ADD', 'IND', 'CLASS',
                                [ind_name, class_name])
        if res.success is False:
            rospy.logwarn("Error %i: %s", res.exit_code, res.error_description)
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding individual %s to class %s: %s", ind_name, class_name, e)
        return None


def add_objectprop_to_ind(client_id, reference_name, objectprop_name, ind_name, value_obj_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        add_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = add_call(client_id, reference_name, 'ADD', 'OBJECTPROP', 'IND',
                       [objectprop_name, ind_name, value_obj_name])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding property %s to individual %s: %s", objectprop_name, ind_name, e)
        return None


def add_dataprop_to_ind(client_id, reference_name, dataprop_name, ind_name, value_type, value):
    rospy.wait_for_service('armor_interface_srv')
    try:
        add_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = add_call(client_id, reference_name, 'ADD', 'DATAPROP', 'IND',
                       [dataprop_name, ind_name, value_type, value])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding property %s to individual %s: %s", dataprop_name, ind_name, e)
        return None


def replace_objectprop_to_ind(client_id, reference_name, objectprop_name, ind_name, new_value, old_value):
    rospy.wait_for_service('armor_interface_srv')
    try:
        replace_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = replace_call(client_id, reference_name, 'REPLACE', 'OBJECTPROP', 'IND',
                           [objectprop_name, ind_name, new_value, old_value])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon adding property %s to individual %s: %s", objectprop_name, ind_name, e)
        return None


def replace_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name, value_type, new_value, old_value):
    rospy.wait_for_service('armor_interface_srv')
    try:
        replace_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = replace_call(client_id, reference_name, 'REPLACE', 'DATAPROP', 'IND',
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


def query_dataprop_b2_ind(client_id, reference_name, dataprop_name, ind_name):
    rospy.wait_for_service('armor_interface_srv')
    try:
        update_shape_call = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
        res = update_shape_call(client_id, reference_name, 'QUERY', 'DATAPROP', 'IND',
                                [dataprop_name, ind_name])
        return res
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed upon querying property %s to individual %s: %s", dataprop_name, ind_name, e)
        return None
