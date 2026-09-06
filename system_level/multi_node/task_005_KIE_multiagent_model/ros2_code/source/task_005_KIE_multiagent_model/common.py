# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import re

try:
    from urlparse import urlparse
except ImportError:
    from urllib.parse import urlparse


IP4_PATTERN = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}")


def get_hostname(url):
    '''
    Extracts the hostname from given url.

    :param str url: the url to parse
    :return: the hostname or `None`, if the url is `None` or `invalid`
    :rtype: str
    :see: http://docs.python.org/library/urlparse.html
    '''
    if url is None:
        return None
    o = urlparse(url)
    hostname = o.hostname
    if hostname is None:
        div_idx = url.find(':')
        if div_idx > -1:
            hostname = url[0:div_idx]
        else:
            hostname = url
    return hostname