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
import random
import enum
import time

DELIMETER = b","
ENDBYTE = b'\r'
STARTBYTE = b"\n"


class LivePlot:
    '''
    Thin wrapper over the default plot to provide an interface for live plotting
    '''

    def __init__(self, data_source,
                 *, title=None,
                 xlabel=None,
                 ylabel=None,
                 max_points=100):
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

        self._axis_1.set_title("a")
        self._axis_1.set_title("b")
        self._axis_1.set_title("c")
        self._axis_1.set_title("d")
        if title: self._axis_1.set_title(title)
        if xlabel: self._axis_1.set_xlabel(xlabel)
        if ylabel: self._axis_1.set_ylabel(ylabel)

    def add_data(self, x, y, z, a, *extra):
        '''
        adds the arbitrary x and y data points to the data set used by the plot
        If adding the data would have the plot exceed max_points, the least
        recently added data point is removed
        '''
        if extra: print(extra)
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
        self.a_data.append(a)

    def _add_data_thread(self, shutdown_event):
        try:
            f = open('data' + str(random.randint(0, 100000)) + '.csv', "w+")
            for new_data in self._data_source:
                if shutdown_event.is_set():
                    f.close()
                    return
                self.add_data(*new_data)
                f.write(str(new_data).replace("(", "\n").replace(")", " ").strip(" "))
                # time.sleep(1 / 60.0)
        except (KeyboardInterrupt, SystemExit):
            f.close()
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
        line_1, = self._axis_1.plot(list(self.x_data), list(self.x_data), 'r*')
        line_2, = self._axis_2.plot(list(self.y_data), list(self.x_data), 'b-')
        line_3, = self._axis_1.plot(list(self.z_data), list(self.x_data), 'r*')
        line_4, = self._axis_2.plot(list(self.a_data), list(self.x_data), 'b-')

        while True:
            try:
                line_1.set_ydata([a for a in list(self.x_data)])
                self._axis_1.clear()
                self._axis_1.plot(range(len(self.x_data)), list(self.x_data), 'r-')

                line_2.set_ydata([a for a in list(self.y_data)])
                self._axis_2.clear()
                self._axis_2.plot(list(self.x_data), list(self.y_data), 'b-')

                line_3.set_ydata([a for a in list(self.z_data)])
                self._axis_3.clear()
                self._axis_3.plot(list(self.x_data), list(self.z_data), 'g-')

                line_4.set_ydata([a for a in list(self.a_data)])
                self._axis_4.clear()
                self._axis_4.plot(list(self.x_data), list(self.a_data), 'r-')

                plt.pause(1.0 / 60.0)
            except (KeyboardInterrupt, SystemExit, Exception) as e:
                shutdown_event.set()
                raise e


def establish_serial(baud_rate=None, serial_path=None):
    while not baud_rate:
        try:
            baud_rate = input("What is the baud rate: ")
            baud_rate = int(baud_rate)
        except (EOFError, ValueError):
            baud_rate = None
            print("Entered baud rate was not a number, please try again")

    if not serial_path:
        ports = list_ports.comports()
        choices = {}
        for i, p in enumerate(ports):
            print(i, end='\t')
            print(p)
            choices[i] = p
        choice = input("Which port: ")
        serial_path = str(choices[int(choice)]).split(' ')[0]

    return BeeConnection(serial_path, baud_rate)


class ParserStates(enum.Enum):
    DATA_READY = "The data is ready to be taken"
    WAITING_FOR_START = "Waiting for a start byte"
    TAKING_NUMBER_AWAITING_SIGN = "Taking number, waiting for +/- info"
    TAKING_NUMBER = "Taking number w/ no decimal"
    TAKING_NUMBER_DECIMAL = "Taking number; encountered decimal pt."
    TOOK_END_BYTE = "Encountered the end byte"
    ERROR_STATE = "ERROR STATE"


class Parser:
    def __init__(self, *, num_data=4, delimeter=DELIMETER, endbyte=ENDBYTE, startbyte=STARTBYTE):
        '''
        FSM data parser for serial data
        '''
        self.num_data = num_data
        self.state = ParserStates.WAITING_FOR_START
        self.data = collections.deque()
        self.number_buffer = collections.deque()
        self.delimeter = delimeter
        self.endbyte = endbyte
        self.startbyte = startbyte
        self.numbers_taken = 0

    def reset(self):
        '''
        Sets the state to WAITING_FOR_START and clears the data
        '''
        self.state = ParserStates.WAITING_FOR_START
        self.data = collections.deque()
        self.number_buffer = []
        self.numbers_taken = 0

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
        negative_sign = b'-'
        number_chars = [b'1', b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9', b'0', decimal_point, negative_sign]

        if self.state == ParserStates.WAITING_FOR_START:
            if char == self.startbyte:
                self.state = ParserStates.TAKING_NUMBER_AWAITING_SIGN
            else:
                return

        elif self.state == ParserStates.TAKING_NUMBER_AWAITING_SIGN:
            if char in number_chars:
                if char == negative_sign:
                    self.number_buffer.append(char)
                    self.state = ParserStates.TAKING_NUMBER
                elif char == decimal_point:
                    self.number_buffer.append(char)
                    self.state = ParserStates.TAKING_NUMBER_DECIMAL
                else:
                    self.number_buffer.append(char)
                    self.state = ParserStates.TAKING_NUMBER
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_NUMBER:
            if char in number_chars:
                self.number_buffer.append(char)
                if char == decimal_point:
                    self.state = ParserStates.TAKING_NUMBER_DECIMAL
            elif char == self.delimeter and not self.numbers_taken >= self.num_data:
                self.numbers_taken += 1
                self.state = ParserStates.TAKING_NUMBER_AWAITING_SIGN
                self._crunch_number_buffer()
            elif char == self.endbyte and self.numbers_taken + 1 == self.num_data:
                self.state = ParserStates.DATA_READY
                self._crunch_number_buffer()
            else:
                self.state = ParserStates.ERROR_STATE

        elif self.state == ParserStates.TAKING_NUMBER_DECIMAL:
            if char in number_chars and char != decimal_point:
                self.number_buffer.append(char)
            elif char == self.delimeter and not self.numbers_taken >= self.num_data:
                self.numbers_taken += 1
                self.state = ParserStates.TAKING_NUMBER_AWAITING_SIGN
                self._crunch_number_buffer()
            elif char == self.endbyte and self.numbers_taken + 1 == self.num_data:
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
        self._connection = serial.Serial(serial_path, baud_rate, timeout=timeout)

        attempts = 0
        while not self._connection.isOpen():
            if attempts == 10:
                raise ConnectionError("Failed to connect to serial device")
            attempts += 1
            time.sleep(0.5)

        self.raw_data = collections.deque()
        self._parser = Parser(num_data=6)

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

    def __iter__(self):
        while True:
            try:
                if self._connection.inWaiting():

                    new_data = self._connection.read()
                    for char in new_data:
                        char = bytes([char])
                        self._parser.feed(char)

                        if self._parser.state == ParserStates.ERROR_STATE:
                            self._parser.reset()
                        elif self._parser.state == ParserStates.DATA_READY:
                            yield tuple(self._parser.data)
                            self._parser.reset()
                        else:
                            pass

            except:
                raise StopIteration


if __name__ == '__main__':
    conn = establish_serial(9600)
    plot = LivePlot(conn)
    plot.plot_forever()


###################################################################################
class TestParser(unittest.TestCase):
    def test_parser_errors(self):
        p = Parser()
        assert p.state == ParserStates.WAITING_FOR_START
        p.feed(STARTBYTE)
        assert p.state == ParserStates.TAKING_NUMBER_AWAITING_SIGN
        p.feed(b'k')
        assert p.state == ParserStates.ERROR_STATE

    def test_parser_simple_full_trip(self):
        p = Parser()
        p.feed(STARTBYTE)
        assert p.state == ParserStates.TAKING_NUMBER_AWAITING_SIGN
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(DELIMETER)
        assert p.state == ParserStates.TAKING_NUMBER_AWAITING_SIGN
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(DELIMETER)
        assert p.state == ParserStates.TAKING_NUMBER_AWAITING_SIGN
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(DELIMETER)
        assert p.state == ParserStates.TAKING_NUMBER_AWAITING_SIGN
        p.feed(b"2")
        assert p.state == ParserStates.TAKING_NUMBER
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
        assert p.state == ParserStates.TAKING_NUMBER_AWAITING_SIGN
        p.feed(b'-')
        assert p.state == ParserStates.TAKING_NUMBER

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
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        p.feed(DELIMETER)
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        p.feed(DELIMETER)
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        p.feed(DELIMETER)
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER
        p.feed(b'.')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        assert p.state == ParserStates.TAKING_NUMBER_DECIMAL
        p.feed(b'4')
        p.feed(ENDBYTE)
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [44.444, 44.444, 44.444, 44.444]
        p.feed(b's')
        assert p.state == ParserStates.DATA_READY
        p.reset()
        assert p.state == ParserStates.WAITING_FOR_START

    def test_round_trips(self):
        p = Parser()
        for char in STARTBYTE + \
                b'32.21' + \
                DELIMETER + \
                b'11' + \
                DELIMETER + \
                b'32' + \
                DELIMETER + \
                b'111.2' + \
                ENDBYTE:
            p.feed(bytes([char]))
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [32.21, 11, 32, 111.2]

    def test_negative_number_cases(self):
        p = Parser()
        for char in STARTBYTE + \
                b'-32.21' + \
                DELIMETER + \
                b'-11' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'111.2' + \
                ENDBYTE:
            p.feed(bytes([char]))
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [-32.21, -11, -.32, 111.2]

    def test_complex_more_numbers(self):
        p = Parser(num_data=7)
        for char in STARTBYTE + \
                b'-32.21' + \
                DELIMETER + \
                b'-11' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'111.2' + \
                ENDBYTE:
            p.feed(bytes([char]))
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [-32.21, -11, -.32, -.32, -.32, -.32, 111.2]

    def test_non_matching_data(self):
        p = Parser(num_data=7)
        for char in STARTBYTE + \
                b'-32.21' + \
                DELIMETER + \
                b'-11' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                ENDBYTE:
            p.feed(bytes([char]))
        assert p.state != ParserStates.DATA_READY
        p.reset()
        p = Parser(num_data=7)
        for char in STARTBYTE + \
                b'-32.21' + \
                DELIMETER + \
                b'-11' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'-.32' + \
                DELIMETER + \
                b'111.2' + \
                ENDBYTE:
            p.feed(bytes([char]))
        assert p.state == ParserStates.DATA_READY
        assert list(p.data) == [-32.21, -11, -.32, -.32, -.32, -.32, 111.2]
