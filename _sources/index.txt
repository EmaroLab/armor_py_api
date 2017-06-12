.. ArmorPy  - A Python ARMOR wrapper documentation master file, created by
   sphinx-quickstart on Wed Jun 15 15:58:50 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

ArmorPy  - Easy Ontology managament in ROS
==========================================

**ArmorPy** is a client library for **ARMOR** - *A ROS Multi Ontology Reference*

`ARMOR <https://github.com/EmaroLab/armor>`_ is a service that can manage multiple ontologies in ROS, allowing you
to query and manipulate ontologies from one or more clients. In this way you can keep all your semantic knowledge
in a single location without giving up flexibility. Also, you can work with ontologies in any language supported by ROS.

**ArmorPy** helps you quickly develop python clients for ARMOR:

- No need to call ARMOR and manually fill the lengthy messages

- Monitors ARMOR service state and calls' responses for steady fault recovery

- Useful macros

To use the library, simply initialize a client with a ID and a reference to work with:

.. code-block:: python

   client = ArmorClient(client_id, reference_name)

Commands are sorted by their purpose (manipulation, query, utility).

You can call the service functions in the following way:

.. code-block:: python

   client.manipulation.some_manipulation_command(args)
   client.query.some_query_command(args)
   client.utils.some_utils_command(args)

That's it!

ArmorPy currently supports only a fraction of ARMOR functions, but coverage will improve over time.
If you are interested, please feel free to contribute to the project!

.. toctree::
   :maxdepth: 2

   armor_api

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

