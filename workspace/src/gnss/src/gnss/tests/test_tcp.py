#!/usr/bin/env python

PKG = 'gnss'
NAME = 'test_tcp'

import unittest

import rostest

class TestTCP(unittest.TestCase):

    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")
        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestTCP)