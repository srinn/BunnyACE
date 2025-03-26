import serial, threading, time, logging, json, struct, queue, traceback
from serial import SerialException

class BunnyAce:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self._name = config.get_name()
        self.event = threading.Event()
        self.lock = False
        if self._name.startswith('ace '):
            self._name = self._name[4:]
        self.variables = self.printer.lookup_object('save_variables').allVariables

        self.serial_name = config.get('serial', '/dev/ttyACM0')
        self.baud = config.getint('baud', 115200)
        extruder_sensor_pin = config.get('extruder_sensor_pin', None)
        toolhead_sensor_pin = config.get('toolhead_sensor_pin', None)
        self.feed_speed = config.getint('feed_speed', 50)
        self.retract_speed = config.getint('retract_speed', 50)
        self.toolchange_retract_length = config.getint('toolchange_retract_length', 100)
        self.toolhead_sensor_to_nozzle_length = config.getint('toolhead_sensor_to_nozzle', None)
        # self.extruder_to_blade_length = config.getint('extruder_to_blade', None)

        self.max_dryer_temperature = config.getint('max_dryer_temperature', 55)

        self._callback_map = {}
        self.park_hit_count = 5
        self._feed_assist_index = -1
        self._request_id = 0
        self._last_assist_count = 0
        self._assist_hit_count = 0
        self._park_in_progress = False
        self._park_is_toolchange = False
        self._park_previous_tool = -1
        self._park_index = -1
        self.endstops = {}

        # Default data to prevent exceptions
        self._info = {
            'status': 'ready',
            'dryer': {
                'status': 'stop',
                'target_temp': 0,
                'duration': 0,
                'remain_time': 0
            },
            'temp': 0,
            'enable_rfid': 1,
            'fan_speed': 7000,
            'feed_assist_count': 0,
            'cont_assist_time': 0.0,
            'slots': [
                {
                    'index': 0,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                },
                {
                    'index': 1,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                },
                {
                    'index': 2,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                },
                {
                    'index': 3,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                }
            ]
        }

        #self._create_mmu_sensor(config, extruder_sensor_pin, "extruder_sensor")
        #self._create_mmu_sensor(config, toolhead_sensor_pin, "toolhead_sensor")
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)
        self.gcode.register_command(
            'ACE_DEBUG', self.cmd_ACE_DEBUG,
            desc='self.cmd_ACE_DEBUG_help')
    def _calc_crc(self, buffer):
        _crc = 0xffff
        for byte in buffer:
            data = byte
            data ^= _crc & 0xff
            data ^= (data & 0x0f) << 4
            _crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
        return _crc

    def _send_request(self, request):
        if not 'id' in request:
            request['id'] = self._request_id
            self._request_id += 1

        payload = json.dumps(request)
        payload = bytes(payload, 'utf-8')

        data = bytes([0xFF, 0xAA])
        data += struct.pack('@H', len(payload))
        data += payload
        data += struct.pack('@H', self._calc_crc(payload))
        data += bytes([0xFE])
        self._serial.write(data)
        self.lock = time.time()

    def _main_eval(self, eventtime):
        while not self._main_queue.empty():
            task = self._main_queue.get_nowait()
            if task is not None:
                task()

        return eventtime + 0.25

    def _reader(self, eventtime):
        while True:
            try:
                raw_bytes = self._serial.read(size=4096)
            except SerialException:
                logging.error("Unable to communicate with the Palette 2")
                return eventtime + 0.1
            if len(raw_bytes) and bytes([0xFE]) in raw_bytes:
                self.gcode.respond_info(str(raw_bytes))
            else:
                break
        return eventtime + 0.1

    def _writer(self, eventtime):
        try:
            def callback(self, response):
                if response is not None:
                    self._info = response['result']

            if not self._queue.empty():
                task = self._queue.get()
                if task is not None:
                    id = self._request_id
                    self._request_id += 1
                    self._callback_map[id] = task[1]
                    task[0]['id'] = id

                    self._send_request(task[0])
            else:
                id = self._request_id
                self._request_id += 1
                self._callback_map[id] = callback
                self._send_request({"id": id, "method": "get_status"})
        except serial.serialutil.SerialException as e:
            logging.info('ACE error: ' + traceback.format_exc())
            # self.printer.invoke_shutdown("Lost communication with ACE '%s'" % (str(e)))
            # return
        except Exception as e:
            logging.info('ACE: Write error ' + str(e))
        return eventtime + 0.1

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        logging.info('ACE: Connecting to ' + self.serial_name)

        # We can catch timing where ACE reboots itself when no data is available from host. We're avoiding it with this hack
        self._connected = False


        self._queue = queue.Queue()
        self._main_queue = queue.Queue()


    def _handle_disconnect(self):
        logging.info('ACE: Closing connection to ' + self.serial_name)
        self._serial.close()
        self._connected = False
        self.reactor.unregister_timer(self.main_timer)

        self._queue = None
        self._main_queue = None


    def send_request(self, request, callback):
        self._info['status'] = 'busy'
        self._queue.put([request, callback])

    def cmd_ACE_DEBUG(self, gcmd):
        for i in range(0, 10):
            try:
                self._serial = serial.Serial(
                    port=self.serial_name,
                    baudrate=self.baud,
                    timeout=0,
                    write_timeout=0)

                if self._serial.isOpen():
                    self._connected = True
                    break
            except serial.serialutil.SerialException:
                time.sleep(0.5)
                continue

        if not self._connected:
            raise ValueError('ACE: Failed to connect to ' + self.serial_name)

        logging.info('ACE: Connected to ' + self.serial_name)
        self.gcode.respond_info(str(self._serial.isOpen()))
        self.main_timer = self.reactor.register_timer(self._writer, self.reactor.NOW)
        self.reader_call = self.reactor.register_timer(self._reader, self.reactor.NOW)



def load_config(config):
    return BunnyAce(config)


