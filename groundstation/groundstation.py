'''
Live plots data recieved over serial
'''
import collections
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports as list_ports
import unittest
import threading
import atexit
import enum
import time

DELIMETER = b"^"
ENDBYTE = b';'
STARTBYTE = b"/"


class LivePlot:
    '''
    Thin wrapper over the default plot to provide an interface for live plotting
    '''

    def __init__(self, data_source,
                 *, title=None,
                 xlabel=None,
                 ylabel=None,
                 max_points=1000):
        self._data_source = data_source
        self._max_points = max_points

        self.x_data = collections.deque([None] * max_points, maxlen=max_points)
        self.y_data = collections.deque([None] * max_points, maxlen=max_points)
        self.z_data = collections.deque([None] * max_points, maxlen=max_points)
        self.a_data = collections.deque([None] * max_points, maxlen=max_points)

        ###################################################################
        # We make the implicit assumption that the data will be displayed #
        # in a 1x1 ratio.                                                 #
        ###################################################################
        self._figure = plt.figure()
        self._axis_1 = self._figure.add_subplot(2, 2, 1)
        self._axis_2 = self._figure.add_subplot(2, 2, 2)
        self._axis_3 = self._figure.add_subplot(2, 2, 3)
        self._axis_4 = self._figure.add_subplot(2, 2, 4)

        if title: self._axis_1.set_title(title)
        if xlabel: self._axis_1.set_xlabel(xlabel)
        if ylabel: self._axis_1.set_ylabel(ylabel)

    def add_data(self, x, y, z, a):
        '''
        adds the arbitrary x and y data points to the data set used by the plot
        If adding the data would have the plot exceed max_points, the least
        recently added data point is removed
        '''
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
        self.a_data.append(a)

    def _add_data_thread(self, shutdown_event):
        try:
            for new_data in self._data_source:
                if shutdown_event.is_set():
                    return
                self.add_data(*new_data)
                time.sleep(0.0001)
        except (KeyboardInterrupt, SystemExit):
            return

    def plot_forever(self):
        '''
        Continuously plots data from the data source
        '''
        #################################################################
        # Since the target model for a live plot is a continuous        #
        # data source, we start a new thread to do that data collection #
        #################################################################
        shutdown_event = threading.Event()
        data_col_thread = threading.Thread(target=self._add_data_thread, args=(shutdown_event,))
        data_col_thread.start()

        def _kill_thread():
            '''
            kills the data collection thread
            '''
            data_col_thread.do_run = False
            data_col_thread.join()

        atexit.register(_kill_thread)

        plt.ion()
        line_1, = self._axis_1.plot(list(self.y_data), list(self.x_data), 'r*')
        line_2, = self._axis_2.plot(list(self.y_data), list(self.x_data), 'b-')
        line_3, = self._axis_1.plot(list(self.y_data), list(self.x_data), 'r*')
        line_4, = self._axis_2.plot(list(self.y_data), list(self.x_data), 'b-')
        while True:
            try:
                line_1.set_ydata([a for a in self.x_data])
                self._axis_1.clear()
                self._axis_1.plot(range(len(self.x_data)), list(self.x_data), 'r-')

                line_2.set_ydata([a for a in self.y_data])
                self._axis_2.clear()
                self._axis_2.plot(range(len(self.y_data)), list(self.y_data), 'b-')

                line_3.set_ydata([a for a in self.z_data])
                self._axis_3.clear()
                self._axis_3.plot(range(len(self.z_data)), list(self.z_data), 'g-')

                line_4.set_ydata([a for a in self.a_data])
                self._axis_4.clear()
                self._axis_4.plot(range(len(self.a_data)), list(self.a_data), 'r-')

                plt.pause(0.00001)
            except (KeyboardInterrupt, SystemExit, Exception) as e:
                shutdown_event.set()
                raise e


'''
import random
def f():
    while True:
        yield random.random(),random.random(),random.random(),random.random()
a = LivePlot(f())
a.plot_forever()
'''


def establish_serial(baud_rate=None, serial_path=None):
    while not baud_rate:
        try:
            baud_rate = input("What is the baud rate: ")
            baud_rate = int(baud_rate)
        except (EOFError, ValueError):
            baud_rate = None
            print("Entered baud rate was not a number, please try again")

    ports = list_ports.comports()
    for p in ports:
        print(p)

    # TODO handle input from user to determine serial path
    serial_path = '/dev/something'

    return BeeConnection(serial_path, baud_rate)


class ParserStates(enum.Enum):
    DATA_READY = "The data is ready to be taken"
    WAITING_FOR_START = "Waiting for a start byte"
    TAKING_FIRST_NUMBER = "In process of taking first number"
    TAKING_FIRST_NUMBER_DOT = "In process of taking first number, encountered decimal point"
    TAKING_SECOND_NUMBER = "In process of taking second number"
    TAKING_SECOND_NUMBER_DOT = "In process of taking second number, encountered decimal point"
    TAKING_THIRD_NUMBER = "In process of taking third number"
    TAKING_THIRD_NUMBER_DOT = "In process of taking third number, encountered decimal point"
    TAKING_FOURTH_NUMBER = "In process of taking fourth number"
    TAKING_FOURTH_NUMBER_DOT = "In process of taking fourth number, encountered decimal point"
    TOOK_END_BYTE = "Encountered the end byte"
    ERROR_STATE = "ERROR STATE"


class Parser:
    def __init__(self, delimeter=b"^", endbyte=b';', startbyte=b"/"):
        '''
        FSM data parser for serial data
        '''
        self.state = ParserStates.WAITING_FOR_START
        self.data = collections.deque()
        self.number_buffer = []
        self.delimeter = delimeter
        self.endbyte = endbyte
        self.startbyte = startbyte

    def reset(self):
        '''
        Sets the state to WAITING_FOR_START and clears the data
        '''
        self.state = ParserStates.WAITING_FOR_START
        self.data = collections.deque()
        self.number_buffer = []

    def _crunch_number_buffer(self):
        '''
        tries to add what is in the number buffer to the data set
        sets the state to an ERROR_STATE if there is a failure
        '''
        try:
            self.data.append(float(b''.join(self.number_buffer)))
            self.number_buffer = []
        except:
            self.state = ParserStates.ERROR_STATE

    def feed(self, char):
        '''
        Feeds a single char to the parser

        TODO refactor out the repetitive parts of this code
        '''
        if len(char) != 1 and self.state != ParserStates.WAITING_FOR_START:
            self.state = ParserStates.ERROR_STATE
            return

        decimal_point = b'.'
        number_chars = [b'1', b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9', b'0', decimal_point]

        if self.state == ParserStates.WAITING_FOR_START:
            if char == self.startbyte:
                self.state = ParserStates.TAKING_FIRST_NUMBER
            else:
                return

        elif self.state == ParserStates.TAKING_FIRST_NUMBER:
            if char in number_chars:
                self.number_buffer.append(char)
                if char == decimal_point:
                    self.state = ParserStates.TAKING_FIRST_NUMBER_DOT
            elif char == self.delimeter:
                self.state = ParserStates.TAKING_SECOND_NUMBER
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_FIRST_NUMBER_DOT:
            if char in number_chars and char != decimal_point:
                self.number_buffer.append(char)
            elif char == self.delimeter:
                self.state = ParserStates.TAKING_SECOND_NUMBER
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_SECOND_NUMBER:
            if char in number_chars:
                self.number_buffer.append(char)
                if char == decimal_point:
                    self.state = ParserStates.TAKING_SECOND_NUMBER_DOT
            elif char == self.delimeter:
                self.state = ParserStates.TAKING_THIRD_NUMBER
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_SECOND_NUMBER_DOT:
            if char in number_chars and char != decimal_point:
                self.number_buffer.append(char)
            elif char == self.delimeter:
                self.state = ParserStates.TAKING_THIRD_NUMBER
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_THIRD_NUMBER:
            if char in number_chars:
                self.number_buffer.append(char)
                if char == decimal_point:
                    self.state = ParserStates.TAKING_THIRD_NUMBER_DOT
            elif char == self.delimeter:
                self.state = ParserStates.TAKING_FOURTH_NUMBER
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_THIRD_NUMBER_DOT:
            if char in number_chars and char != decimal_point:
                self.number_buffer.append(char)
            elif char == self.delimeter:
                self.state = ParserStates.TAKING_FOURTH_NUMBER
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_FOURTH_NUMBER:
            if char in number_chars:
                self.number_buffer.append(char)
                if char == decimal_point:
                    self.state = ParserStates.TAKING_FOURTH_NUMBER_DOT
            elif char == self.endbyte:
                self.state = ParserStates.DATA_READY
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_FOURTH_NUMBER_DOT:
            if char in number_chars and char != decimal_point:
                self.number_buffer.append(char)
            elif char == self.endbyte:
                self.state = ParserStates.DATA_READY
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE


class BeeConnection:
    '''
    Iterator that represents a view of the data being sent over serial
    by the exBee
    '''

    def __init__(self, serial_path, baud_rate, timeout=1):
        '''
        initializes serial connection. If initial connect fails, waits half
        a second and tries again. If the connection still is not established
        after 10 such additional attempts, raises ConnectionError
        '''
        self._connection = serial.Serial(serial_path, baud_rate, timeout)

        attempts = 0
        while not self._connection.isOpen():
            if attempts == 10:
                raise ConnectionError("Failed to connect to serial device")
            attempts += 1
            time.sleep(0.5)

        self.raw_data = collections.deque()
        self._parser = Parser()

    def close(self):
        '''
        cleans up and closes the serial connection
        '''
        if self._connection.isOpen():
            self._connection.flush()
            self._connection.close()

        if not self._connection.isOpen():
            print("Serial port successfully closed")
        else:
            print("Something went wrong closing the connection")

    def __exit__(self):
        '''
        closes connection when the object is used in a with block
        '''
        self.close()

    def __del__(self):
        '''
        closes connection if the object is deleted
        '''
        self.close()

    def __next__(self):
        try:
            if self._connection.inWaiting():
                new_data = self._connection.read()
                for char in new_data:
                    self._parser.feed(char)

                    if self._parser.state == ParserStates.ERROR_STATE:
                        self._parser.reset()
                    elif self._parser.state == ParserStates.DATA_READY:
                        yield self._parser.data
                        self._parser.reset()
                    else:
                        pass



        except:
            raise StopIteration

    def __iter__(self):
        return self


###################################################################################
class TestParser(unittest.TestCase):
    def test_parser_errors(self):
        p = Parser()
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(STARTBYTE)
        assert p.state == ParserStates.TAKING_FIRST_NUMBER
        p.feed(b'k')
        assert p.state == ParserStates.ERROR_STATE

    def test_parser_simple_full_trip(self):
        p = Parser()
        p.feed(STARTBYTE)
        assert p.state == ParserStates.TAKING_FIRST_NUMBER
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_FIRST_NUMBER
        p.feed(DELIMETER)
        assert p.state == ParserStates.TAKING_SECOND_NUMBER
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_SECOND_NUMBER
        p.feed(DELIMETER)
        assert p.state == ParserStates.TAKING_THIRD_NUMBER
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_THIRD_NUMBER
        p.feed(DELIMETER)
        assert p.state == ParserStates.TAKING_FOURTH_NUMBER
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_FOURTH_NUMBER
        p.feed(ENDBYTE)
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [2.0, 2.0, 2.0, 2.0]

    def test_parser_wait_for_start(self):
        p = Parser()
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(b'JUNK')
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(b'J')
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(b'RR')
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(DELIMETER)
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(ENDBYTE)
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(STARTBYTE)
        assert p.state == ParserStates.TAKING_FIRST_NUMBER

    def test_parser_complex_trip(self):
        p = Parser()
        p.feed(STARTBYTE)
        p.feed(b'2')
        p.feed(b'9')
        p.feed(b'.')
        p.feed(b'1')
        p.feed(DELIMETER)
        p.feed(b'1')
        p.feed(b'2')
        p.feed(b'3')
        p.feed(b'4')
        p.feed(b'5')
        p.feed(b'6')
        p.feed(b'7')
        p.feed(b'8')
        p.feed(b'9')
        p.feed(b'0')
        p.feed(DELIMETER)
        p.feed(b'3')
        p.feed(b'.')
        p.feed(DELIMETER)
        p.feed(b'1')
        p.feed(b'5')
        p.feed(ENDBYTE)
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [29.1, 1234567890.0, 3.0, 15.0]

    def test_parser_decimal_read(self):
        p = Parser()
        p.feed(STARTBYTE)
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FIRST_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FIRST_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_FIRST_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FIRST_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FIRST_NUMBER_DOT
        p.feed(b'4')
        p.feed(DELIMETER)
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_SECOND_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_SECOND_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_SECOND_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_SECOND_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_SECOND_NUMBER_DOT
        p.feed(b'4')
        p.feed(DELIMETER)
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_THIRD_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_THIRD_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_THIRD_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_THIRD_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_THIRD_NUMBER_DOT
        p.feed(b'4')
        p.feed(DELIMETER)
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FOURTH_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FOURTH_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_FOURTH_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FOURTH_NUMBER_DOT
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_FOURTH_NUMBER_DOT
        p.feed(b'4')
        p.feed(ENDBYTE)
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [44.444, 44.444, 44.444, 44.444]
        p.feed(b's')
        assert p.state == ParserStates.DATA_READY
        p.reset()
        assert p.state == ParserStates.WAITING_FOR_START
