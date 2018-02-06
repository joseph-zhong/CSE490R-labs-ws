#!/usr/bin/env python

from pprint import pprint

class FeedbackController(object):
    def __init__(self):
        pass

    def image_cb(self, msg):
        pprint(msg)
