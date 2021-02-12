#!/usr/bin/env python

import unittest


PKG = 'twizy_control'


class TestCAN(unittest.TestCase):

    # Unittests only runs class functions starting with test_
    def test_dummy(self):
        self.assertEqual(1, 1, '1 != 1')


def main():
    # Start tests
    rostest.rosrun(PKG, NAME, TestCAN)


if __name__ == '__main__':
    main()
