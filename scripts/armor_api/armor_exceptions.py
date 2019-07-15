#!/usr/bin/env python

"""
Exceptions raised by Armor Python API --ArmorPy
"""

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"


class ArmorPyException(Exception):
    def __init__(self, message):
        self.msg = message
        super(ArmorPyException, self).__init__(self.msg)


class ArmorServiceCallError(ArmorPyException):
    def __init__(self, message):
        self.msg = message
        super(ArmorServiceCallError, self).__init__(self.msg)


class ArmorServiceInternalError(ArmorPyException):
    def __init__(self, message, exit_code):
        self.msg = message
        self.code = exit_code
        err_msg = "ARMOR internal error %s: %s" % (self.code, self.msg)
        super(ArmorServiceInternalError, self).__init__(err_msg)
