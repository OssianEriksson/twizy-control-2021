#!/usr/bin/env python

import argparse

import socket
import select
import random

import rospy

from sbp.navigation import MsgPosLLH


DESCRIPTION = 'Generates random MSG_POS_LLH messages'


def generate_message(reference_time):
    """
    Generates a random MSG_POS_LLH message with a tow value relative to
    the provided reference_time
    """

    tow = int(round((rospy.get_time() - reference_time) * 1000))
    lat = random.uniform(-80, 84)  # Not -90 to 90 so UTM conversion can happen
    lon = random.uniform(-180, 180)
    height = random.uniform(-10000, 10000)
    h_accuracy = random.randint(0, 10000)
    v_accuracy = random.randint(0, 10000)
    n_sats = random.randint(1, 5)

    return MsgPosLLH(sender=1, tow=tow, lat=lat, lon=lon, height=height,
                     h_accuracy=h_accuracy, v_accuracy=v_accuracy,
                     n_sats=n_sats, flags=0)


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument('--port', type=int, default=55555,
                        help='Port to run tests on')
    parser.add_argument('--rate', type=float,
                        default=10,
                        help='Rate to create messages at (Hz)')
    args, unknown = parser.parse_known_args()

    # Initialize ROS node
    rospy.init_node('random_llh')

    # Open TCP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Allow reuse of old sockets still in TIME_WAIT state
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the socket to the given port, an empty string as first element
    # in the tuple signals that this should be a server socket (for a
    # client socket this element would have specified an ip to connect to)
    sock.bind(('', args.port))

    # Make the socket ready to accept connections. Five clients can be in the
    # queue before connections get rejected
    sock.listen(5)

    clients = []

    def connect(client):
        """
        Connect a client with an already opened socket to the server
        """

        clients.append(client)

    def disconnect(client):
        """
        Disconnect (and close) a client from the server
        """

        client.close()
        clients.remove(client)

    rate = rospy.Rate(args.rate)
    reference_time = rospy.get_time()

    while not rospy.is_shutdown():
        # Filter for what sockets are ready for what operations
        readable, writable, errored = select.select(
            [sock], clients, clients, 0)

        # Accept new connections
        if sock in readable:
            client_socket, address = sock.accept()
            connect(client_socket)

        # Generate a MSG_POS_LLH message
        msg = generate_message(reference_time)
        data = msg.to_binary()

        # Send message to clients
        for client in writable:
            try:
                # This is not really responisble, socket.send can fail to send
                # the entire message (the function returns the number of bytes
                # sent). A more sound implementation would save the bytes which
                # failed to be sent and try to send them again next time
                # around.
                client.send(data)
            except (socket.error, RuntimeError):
                # Close clients where writes failed
                disconnect(client)

        # Close clients which have recieved an error
        for client in errored:
            disconnect(client)

        rate.sleep()


if __name__ == '__main__':
    main()
