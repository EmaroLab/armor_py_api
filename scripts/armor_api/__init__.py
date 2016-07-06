"""
The **ArmorPy** armor_api module.

Includes three commands submodules:

    + **Manipulation** - commands that can modify the information stored in an ontology.

    + **Query** - commands used to query information from the ontology.

    + **Sysutil** - utility commands such as load/save ontology, toggle ARMOR logging, etc.

Also includes:

    + **Exceptions** - All exceptions armor_api commands can possibly raise.
"""

from . import manipulation, query, sysutil, exceptions, client

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"

__all__ = ['manipulation', 'query', 'sysutil', 'exceptions']

