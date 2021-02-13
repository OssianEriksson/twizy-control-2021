#!/usr/bin/env python

import argparse

import os
import socket
import select
import threading
import random

import unittest
import rospy
import rostest
import roslaunch
import rospkg

from std_msgs.msg import String
from gnss.msg import GNSSLatLongHeight

from sbp.navigation import MsgPosLLH

from collections import namedtuple

PKG = 'gnss'
NAME = 'test_tcp'
DESCRIPTION = 'Test TCP publisher against 2020\'s gnss data publisher'

SUBSCRIBER_QUEUE_SIZE = 100

SYNC_LAT = 100000


SimplePosition = namedtuple('SimplePosition', ['lon', 'lat'])


class Client:
    """
    Simple network client class
    """

    def __init__(self, socket):
        self.socket = socket
        self.messages = bytearray()

    def send(self, message):
        """
        Queue a message for transmission to the remote client
        """

        self.messages.extend(message)

    def transmit_all(self):
        """
        Attempts to transmit entire backlog of messages. Might only transmit
        parts of it, in which case the parts of the messages that weren't sent
        are kept in the backlog of messages to be sent
        """

        if len(self.messages) > 0:
            bytes_sent = self.socket.send(self.messages)

            if bytes_sent == 0:
                raise RuntimeError('Socket connection broken')

            self.messages = self.messages[bytes_sent:]

    def close(self):
        """
        Close client
        """
        self.socket.close()

    # Function neccesary to be able to pass class to select.select
    def fileno(self):
        return self.socket.fileno()


class Server:
    """
    Simple non-blocking TCP server
    """

    def __init__(self):
        self.clients = []

    def _connect(self, client):
        """
        Connect a client with an already opened socket to the server
        """

        self.clients.append(client)

    def _disconnect(self, client):
        """
        Disconnect (and close) a client from the server
        """

        client.close()
        self.clients.remove(client)

    def start(self, port, queue_size=5):
        """
        Open up the server socket (TCP protocol) and bind it to the given port
        """

        # Open TCP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Allow reuse of old sockets still in TIME_WAIT state
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Bind the socket to the given port, an empty string as first element
        # in the tuple signals that this should be a server socket (for a
        # client socket this element would have specified an ip to connect to)
        self.socket.bind(('', port))

        # Make the socket ready to accept connections. Five clients
        self.socket.listen(queue_size)

    def perform_io(self):
        """
        Accept new connections, do one attempt per client to send out its
        respective backlog of messages and close errored connections
        """

        # Filter for what sockets are ready for what operations
        readable, writable, errored = select.select(
            [self], self.clients, self.clients, 0)

        # Accept new connections
        if self in readable:
            client_socket, address = self.socket.accept()
            self._connect(Client(client_socket))

        # All clients attempt to send out their backlog of messages
        for client in writable:
            try:
                client.transmit_all()
            except (socket.error, RuntimeError):
                # Close clients where writes failed
                self._disconnect(client)

        # Close clients which have recieved an error
        for client in errored:
            self._disconnect(client)

    def send_to_all(self, message):
        """
        Sends the same message to all connected clients
        """

        for client in self.clients:
            client.send(message)

    def close(self):
        """
        Disconnect all clients and close server
        """

        # Disconnect all clients
        for client in self.clients:
            self._disconnect(client)

        # Close server socket
        self.socket.close()

    # Function neccesary to be able to pass class to select.select
    def fileno(self):
        return self.socket.fileno()


class BufferSubscriber:
    """
    An encapsulation of a ROS subscriber for GNSS positioning messages,
    buffering longitude and latitude data of incoming messages.

    The callback function is called with new ROS-messages recieved by the
    subscriber. The callback is expected to call _record_message() with
    messages parsed as a SimplePosition. This stores the message to the 
    internal message buffer. The callback is also assumed to make sure all
    messages provided to _record_message() are not already in the message
    buffer, for example by checking if msg not in BufferSubscriber._messages.

    The ready flag of the BufferSubscriber is set once a SYNC_LAT message is
    recorded in the _record_message() function. A SYNC_LAT message simply has
    a latitude value of SYNC_LAT. These messages do not get stored in the
    internal buffer.
    """

    def __init__(self, topic, type, callback):
        self.registered = True
        self.ready = False
        self._lock = threading.Lock()
        self._messages = []

        def cb(msg):
            # Only accept new messages if we are registred. This prevents
            # potential async access to the message buffer after unregister()
            # has been called, at which point we expect no further async access
            # to the message buffer
            if self.registered:
                callback(msg)

        self.subscriber = rospy.Subscriber(topic, type, callback=cb,
                                           queue_size=SUBSCRIBER_QUEUE_SIZE)

    def _record_message(self, msg):
        """
        Acknowledge that a message has been recieved by this subscriber.
        This method should be called from the callback function (provided to
        this instance in our constructor) of subclasses of this class
        """

        # If we recieved a SYNC_LAT message, don't store it in the message
        # buffer. Instead set the ready flag to True, indicating that the
        # publising node on the other end is up and running
        if abs(msg.lat - SYNC_LAT) > 1e-7:
            with self._lock:
                self._messages.append(msg)
        else:
            self.ready = True

    def msgs_in_buffer(self):
        """
        Returns the current number of messages in the message buffer
        """

        with self._lock:
            return len(self._messages)

    @property
    def messages(self):
        """
        Buffer of unique messages recieved by this subscriber

        unregister() must be called before accessing messages, since while the
        subscriber is registred the message buffer is written to asynchronously
        which makes it require a thread lock. When the message buffer is
        exposed through this property, we don't want to have to use a lock, so
        instead we require that the subscriber is unregistred first, preventing
        further asynchronous access to it
        """

        if self.registered:
            raise RuntimeError('unregister() must be called before accessing '
                               'messages')

        return self._messages

    def unregister(self):
        """
        Unregister subscriber
        """

        self.subscriber.unregister()
        self.registered = False


class BufferSubscriberNew(BufferSubscriber):
    """
    A BufferSubscriber for messages published by current TCP GNSS code
    """

    def __init__(self, topic, type):
        def callback(msg):
            # We only keep longitude and latitude data since this is the only
            # data published by 2020's code which we are interested in
            # comparing against
            self._record_message(SimplePosition(msg.longitude, msg.latitude))

        BufferSubscriber.__init__(self, topic, type, callback)


class BufferSubscriberOld(BufferSubscriber):
    """
    A BufferSubscriber for messages published by 2020's GNSS code (old)
    """

    def __init__(self, topic, type):
        def callback(msg):
            # The 2020 GNSS code will publish some empty data, filter it
            if msg.data:
                pos = SimplePosition(*(float(f) for f in msg.data.split(',')))

                # Prevent duplicates
                if pos not in self._messages:
                    self._record_message(pos)

        BufferSubscriber.__init__(self, topic, type, callback)


class TestTCP(unittest.TestCase):

    def _wait_for_condition(self, condition, timeout):
        """
        Waits for condition to return True, at which point this function will
        also return True. If time passes timeout however, False is returned.

        During the waiting period this function makes continuous calls to
        self.server.perform_io (twice a second)
        """

        t = rospy.get_time()
        while not condition():
            # Return false once the timeout has passed
            if rospy.get_time() > t + subscriber_timeout:
                return False

            self.server.perform_io()

            rospy.sleep(0.5)

        return True

    def _create_buffer_subscribers(self):
        """
        Creates dict of BufferSubscribers subscribed to topics published by 
        nodes to test (with keys new_*) and to test against (with keys old_*).

        This function also makes sure that the nodes publishing to the 
        subscribed topics are up and running before returning by sending out 
        SYNC_LAT messages during a syncing period. (SYNC_LAT messages simply
        have a lat (latitude) value of SYNC_LAT to identify them.) Once the
        node publishes a SYNC_LAT message back which is then read by one of
        the subscribers we know that the node on the other end is online.
        """

        subscribers = {
            'old_l': BufferSubscriberOld('/old/gps_l/GPS_left', String),
            'old_r': BufferSubscriberOld('/old/gps_r/GPS_right', String),

            'new_l': BufferSubscriberNew('/new/gps_l/gnss_llh', GNSSLatLongHeight),
            'new_r': BufferSubscriberNew('/new/gps_r/gnss_llh', GNSSLatLongHeight),
        }

        def subscribers_ready():
            """
            Wait for all subscribers to have thier ready flag set to True. This
            flag is set once the subscriber has recieved atleast one SYNC_LAT
            message

            This function also stages new SYNC_LAT messages for transmission to
            the TCP clients whenever it is called. New messages need to be
            continuously sent out during the syncing period so that nodes that
            have newly come online can recieve such a message and send it back
            for us to know that the node is up and running
            """

            # We continuously send out new SYNC_LAT messages
            msg = MsgPosLLH(sender=1, tow=0, lat=SYNC_LAT, lon=0, height=0,
                            h_accuracy=0, v_accuracy=0, n_sats=1, flags=0)

            self.server.send_to_all(msg.to_binary())

            # All four clients must have connected to the server before we can
            # be ready
            if len(self.server.clients) < 4:
                return False

            for name, subscriber in subscribers.items():
                if not subscriber.ready:
                    return False

            return True

        # Wait for all subscribers to recieve SYNC_LAT messages: Once they have
        # recieved a such message, we know that the node on the other end is up
        # and running since it has been able to respond
        if not self._wait_for_condition(subscribers_ready, connect_timeout):
            self.fail(('Nodes did not connect to server in time. Set timeout '
                       'with the --connect-timeout option. The current '
                       'value is {}').format(connect_timeout))

        return subscribers

    # Called before tests are run
    def setUp(self):
        # Initialize ROS node
        rospy.init_node('test_tcp')

        # Initialize TCP server
        self.server = Server()

        # Start the server on the given port, a maximum of four clients can
        # wait in queue to be accepted by the server before the server starts
        # rejecting connections
        self.server.start(port, 4)

        # Find path to this package
        try:
            pkg_path = rospkg.RosPack().get_path(PKG)
        except rospkg.common.ResourceNotFound:
            raise RuntimeError(('Unable to locate package: {}. '
                                'Have you called "source devel/setup.bash" in '
                                'the catkin workspace?').format(PKG))

        # Launch GPS code from 2020's parallel parking
        launch_path = os.path.join(pkg_path, 'launch', 'old.launch')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        args = ['host_l:=127.0.0.1', 'port_l:={}'.format(port),
                'host_r:=127.0.0.1', 'port_r:={}'.format(port)]
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.parent = roslaunch.parent.ROSLaunchParent(
            uuid, [(launch_path, args)])
        self.launch.start()

        # Launch new TCP GNSS node, once for left...
        rospy.set_param('/new/gps_l/host', '127.0.0.1')
        rospy.set_param('/new/gps_l/port', port)
        self.launch.launch(roslaunch.core.Node(
            PKG, 'tcp', name='gps_l', namespace='new'))

        # ...and once for right
        rospy.set_param('/new/gps_r/host', '127.0.0.1')
        rospy.set_param('/new/gps_r/port', port)
        self.launch.launch(roslaunch.core.Node(
            PKG, 'tcp', name='gps_r', namespace='new'))

    # Unittests only runs class functions starting with test_
    def test_functions_same_as_2020(self):
        """
        Compare the output of new and old code given the same input
        """

        # Initialize subscribers
        subscribers = self._create_buffer_subscribers()

        messages_to_send = 100  # Number of messages to perform tests on

        # Generate some random messages and send them out to the TCP clients
        t = rospy.get_time()
        for i in range(0, messages_to_send):
            tow = int(round((rospy.get_time() - t) * 1000))
            lat = (180.0 * i) / messages_to_send - 90  # At least of lon or lat
                                                       # needs to be unique so
                                                       # we can safely filter
                                                       # duplicate messages
                                                       # later
            lon = random.uniform(-180, 180)
            height = random.uniform(-10000, 10000)
            h_accuracy = random.randint(0, 10000)
            v_accuracy = random.randint(0, 10000)
            n_sats = random.randint(1, 5)

            msg = MsgPosLLH(sender=1, tow=tow, lat=lat, lon=lon, height=height,
                            h_accuracy=h_accuracy, v_accuracy=v_accuracy,
                            n_sats=n_sats, flags=0)

            self.server.send_to_all(msg.to_binary())
            self.server.perform_io()

            # This delay gives time for the publisher queue on the recieving
            # nodes end to be cleared out. Setting delay too low can lead to
            # messages beeing dropped, and therefore failed tests
            rospy.sleep(delay)

        def recieved_messages():
            """
            Checks if all subscribers have recieved all messages that have been
            sent out by the simulated GNSS reciever
            """

            for name, subscriber in subscribers.items():
                # If we have recieved less messages than the total number of
                # messages that were sent out, we still have messages to
                # recieve
                if subscriber.msgs_in_buffer() < messages_to_send:
                    return False

            return True

        # Wait for all messages to arrive back from the nodes we are testing
        if not self._wait_for_condition(recieved_messages, subscriber_timeout):
            self.fail(('Messages did not arrive to subscribers in time. Set '
                       'timeout with the --subscriber-timeout option. The '
                       'current value is {}').format(subscriber_timeout))

        # Unregister subscribers. This prevents further writes from callbacks
        # to their internal buffer, meaning we no longer have to use thread
        # locks to access BufferSubscriber. In fact BufferSubscriber prevents
        # direct access to BufferSubscriber.messages before the subscriber has
        # been unregistred
        for name, subscriber in subscribers.items():
            subscriber.unregister()

        def compare_messages(old, new):
            """
            Compare messages collected by two subscribers: One with messages in
            the format used in 2020 (old) and one in the current format (new)
            """

            old_msgs = subscribers[old].messages
            new_msgs = subscribers[new].messages

            # Messages are guaranteed to arrive in order
            for old_msg, new_msg in zip(old_msgs, new_msgs):
                # Only compare latitude and longitude since this is the only
                # data provided by old code
                self.assertAlmostEqual(
                    old_msg.lon, new_msg.lon, msg='Longitude disagreement')
                self.assertAlmostEqual(
                    old_msg.lat, new_msg.lat, msg='Latitude disagreement')

        # Compare messages collected from old and new code
        compare_messages('old_l', 'new_l')

    # Called after all tests have been run, succesful or not
    def tearDown(self):
        # Shut down nodes
        try:
            self.launch.shutdown()
        except:
            pass

        # Close TCP server after tests finish
        try:
            self.server.close()
        except:
            pass


def main():
    global port, connect_timeout, subscriber_timeout, delay

    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument('--port', type=int, default=55555,
                        help='Port to run tests on')
    parser.add_argument('--connect-timeout', type=float,
                        default=10,
                        help='Time limit for nodes to connect to server '
                        'before failing tests')
    parser.add_argument('--subscriber-timeout', type=float,
                        default=10,
                        help='Time limit for subscribers to collect messages '
                        'before failing tests')
    parser.add_argument('--delay', type=float, default=0.1,
                        help='Delay between consecutive releases of data from '
                        'simulated GNSS reciever')
    args, unknown = parser.parse_known_args()

    # Set some properties
    port = args.port
    connect_timeout = args.connect_timeout
    subscriber_timeout = args.subscriber_timeout
    delay = args.delay

    # Start tests
    rostest.rosrun(PKG, NAME, TestTCP)


if __name__ == '__main__':
    main()
