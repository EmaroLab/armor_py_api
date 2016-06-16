.. Pyarmor - A Python ARMOR wrapper documentation master file, created by
   sphinx-quickstart on Wed Jun 15 15:58:50 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Pyarmor - Easy Ontology managament in ROS
=========================================

**Pyarmor** is a wrapper library for **ARMOR** - *A ROS multi ontology reference*

**ARMOR** is a service that can manage multiple ontologies in ROS, allowing you
to query and manipulate ontologies from one or more clients, independently of the
language they are coded in. In thi way you can keep all your semantic knowledge
in a single location without giving up flexibility. You can also avoid writing a single
code of Java.

**Pyarmor** helps you speed up the development process of any node which is supposed to
work with ARMOR:

- No need to manually call ARMOR and compile the lengthy messages! Everything is done inside Pyarmor high-level functions.

- Higher level utility functions for common operations actually performs multiple calls to ARMOR and returns the desired results, keeping your code clean and tidy.

- Monitors ARMOR results and service state for steady fault recovery.


**Check it out!**

.. toctree::
   :maxdepth: 1

   pyarmor

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

