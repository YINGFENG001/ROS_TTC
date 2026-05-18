from __future__ import annotations

import queue
import threading
from typing import Callable, Optional

try:
    import serial
except ImportError:  # pragma: no cover - reported when the node starts.
    serial = None

from .protocol import Stm32Message, format_command, parse_protocol_line, response_matches


RawLineCallback = Callable[[str], None]
MessageCallback = Callable[[Stm32Message], None]


class Stm32CommandError(RuntimeError):
    def __init__(self, message: str, response: Optional[Stm32Message] = None) -> None:
        super().__init__(message)
        self.response = response


class Stm32SerialClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
        raw_rx_callback: Optional[RawLineCallback] = None,
        raw_tx_callback: Optional[RawLineCallback] = None,
        message_callback: Optional[MessageCallback] = None,
    ) -> None:
        if serial is None:
            raise RuntimeError('pyserial is not installed')

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.raw_rx_callback = raw_rx_callback
        self.raw_tx_callback = raw_tx_callback
        self.message_callback = message_callback

        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=timeout,
        )
        self._write_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._id_lock = threading.Lock()
        self._next_id = 1
        self._responses: dict[int, queue.Queue[Stm32Message]] = {}
        self._closed = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, name='stm32-serial-reader', daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._closed.set()
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        finally:
            if self._reader.is_alive():
                self._reader.join(timeout=0.5)

    def send_command(
        self,
        command: str,
        wait_for: str = 'ack',
        timeout: Optional[float] = None,
    ) -> Stm32Message:
        if wait_for not in {'ack', 'done', 'state'}:
            raise ValueError(f'unsupported wait_for type: {wait_for}')

        command_id = self._allocate_id()
        line = format_command(command_id, command)
        response_queue: queue.Queue[Stm32Message] = queue.Queue()
        actual_timeout = self.timeout if timeout is None else timeout

        with self._command_lock:
            self._responses[command_id] = response_queue
            try:
                self._write_line(line)
                return self._wait_for_response(command_id, response_queue, wait_for, actual_timeout)
            finally:
                self._responses.pop(command_id, None)

    def _allocate_id(self) -> int:
        with self._id_lock:
            value = self._next_id
            self._next_id += 1
            if self._next_id > 999999:
                self._next_id = 1
            return value

    def _write_line(self, line: str) -> None:
        payload = (line.rstrip('\r\n') + '\n').encode('utf-8')
        with self._write_lock:
            self._serial.write(payload)
            self._serial.flush()
        if self.raw_tx_callback:
            self.raw_tx_callback(line)

    def _wait_for_response(
        self,
        command_id: int,
        response_queue: queue.Queue[Stm32Message],
        wait_for: str,
        timeout: float,
    ) -> Stm32Message:
        while True:
            try:
                message = response_queue.get(timeout=timeout)
            except queue.Empty as exc:
                raise TimeoutError(f'timeout waiting for @{wait_for} id={command_id}') from exc

            if message.type == 'err':
                code = message.fields.get('code', 'unknown')
                detail = message.fields.get('detail', '')
                suffix = f' detail={detail}' if detail else ''
                raise Stm32CommandError(f'STM32 error id={command_id} code={code}{suffix}', message)

            if response_matches(message, wait_for):
                return message

    def _read_loop(self) -> None:
        while not self._closed.is_set():
            try:
                raw = self._serial.readline()
            except Exception:
                if self._closed.is_set():
                    return
                continue

            if not raw:
                continue

            try:
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception:
                continue

            if not line:
                continue

            message = parse_protocol_line(line)
            if message is None:
                continue

            if self.raw_rx_callback:
                self.raw_rx_callback(message.raw)
            if self.message_callback:
                self.message_callback(message)

            command_id = message.id
            if command_id is None:
                continue
            response_queue = self._responses.get(command_id)
            if response_queue is not None:
                response_queue.put(message)
