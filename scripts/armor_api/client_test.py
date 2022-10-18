#! /usr/bin/env python
from armor_client import ArmorClient
from os.path import dirname, realpath

# INITIALIZE REFERENCE

path = dirname(realpath(__file__))
path = path + "/../../test/"

client = ArmorClient("client", "reference")
client.utils.load_ref_from_file(path + "test.owl", "http://www.semanticweb.org/emarolab/pyarmor/test",
                                True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

# ADD SOME AXIOMS

client.manipulation.add_ind_to_class("ind_1", "Class_1")
print("Added ind_1 to Class_1")
client.manipulation.add_ind_to_class("ind_2", "Class_2")
print("Added ind_2 to Class_2")
client.manipulation.add_ind_to_class("ind_3", "Class_3")
print("Added ind_3 to Class_3")

client.manipulation.add_objectprop_to_ind("OProperty_1", "ind_1", "ind_3")
client.manipulation.add_objectprop_to_ind("OProperty_2", "ind_1", "ind_2")

client.manipulation.add_dataprop_to_ind("DProperty_1", "ind_1", "INTEGER", "1")

client.manipulation.replace_dataprop_b2_ind("DProperty_1", "ind_1", "INTEGER", "3", "1")

# APPLY CHANGES AND QUERY

client.utils.apply_buffered_changes()
client.utils.sync_buffered_reasoner()

if client.query.check_ind_exists("ind_1"):
    print("Success! ind_1 is in Class_4 now!")

# SAVE AND EXIT

client.utils.save_ref_with_inferences(path + "inferred_test.owl")



