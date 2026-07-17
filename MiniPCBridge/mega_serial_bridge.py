#!/usr/bin/env python3
"""
mega_serial_bridge.py -- mini PC serial<->network bridge for the Feel
Festival wave motor control system.

Runs on the backstage mini PC, sitting between the two Arduino Megas
(LEFT/RIGHT, each on its own USB serial port) and the main PC, which runs
TouchDesigner and does all the choreography/visualization/PID-tuning work
over there (see TouchdDesignerCode/WaveMotorControl/extensions/
SerialRelayEXT.py for the TD side of this link).

This script has ZERO awareness of the MOTOR protocol (SETPOS/HOME/etc) --
it just relays newline-terminated ASCII lines verbatim, in both
directions, between each Mega's serial port and a matching TCP port that
the main PC connects to. All motor protocol logic (motor ids, command
formatting, message parsing) lives on the main PC, in
SerialProtocolBase.py. If the Mega's wire protocol ever changes, this file
shouldn't need to.

It DOES have its own small, separate vocabulary of self-status messages,
prefixed "BRIDGE " so they can never collide with anything the Mega itself
sends (the Mega's messages are READY/POS/HOMED/FAULT/STATUS/PID_UPDATED/
PID_RESET/ERR -- none start with "BRIDGE"). These exist because a TCP
connection succeeding only means the main PC reached the BRIDGE -- it says
nothing about whether the bridge's own serial link to the Mega is actually
up, which was a real source of confusion before this existed:

    BRIDGE CONNECTED SERIAL_UP    -- sent right when a client connects, if
                                      the Mega serial link is up at that moment
    BRIDGE CONNECTED SERIAL_DOWN  -- same, but the serial link is down
    BRIDGE SERIAL_UP              -- serial link came up (or came back up)
                                      while a client was already connected
    BRIDGE SERIAL_DOWN            -- serial link dropped while a client was
                                      connected
    BRIDGE DISCONNECTING <reason> -- sent to a client just before this
                                      bridge deliberately closes their
                                      connection (e.g. displaced by a new
                                      one), so it's distinguishable from an
                                      unexplained connection reset

Requires: pyserial (`pip install pyserial`)

Usage:
    python mega_serial_bridge.py

Edit the CONFIG section below for your COM ports before running. Designed
to run indefinitely: recovers from a disconnected Mega (keeps retrying the
serial port) and from a disconnected/restarted main PC (keeps accepting a
fresh TCP connection) without needing to be restarted itself.
"""

import logging
import socket
import threading
import time

import serial


# ---------------------------------------------------------------------------
# CONFIG -- edit for your installation
# ---------------------------------------------------------------------------

BAUD_RATE = 115200  # must match BAUD_RATE in motor_controller.ino

SIDES = [
    {'name': 'LEFT',  'serial_port': 'COM10', 'tcp_port': 9000},
    {'name': 'RIGHT', 'serial_port': 'COM11', 'tcp_port': 9001},
]

TCP_HOST = '0.0.0.0'          # listen on all interfaces
SERIAL_RETRY_SECONDS = 2.0    # how often to retry opening a disconnected Mega
TCP_RECV_BUFSIZE = 4096

# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(message)s',
    datefmt='%H:%M:%S',
)


class MegaBridge:
    """
    Bridges one Mega's serial port to one TCP port. Fully bidirectional,
    line-oriented, protocol-agnostic -- forwards bytes, nothing else.
    Only one TCP client at a time; a new connection replaces any previous
    one (the main PC reconnecting after a restart just works).
    """

    def __init__(self, name, serial_port, baud_rate, tcp_host, tcp_port):
        self.name = name
        self.serial_port_name = serial_port
        self.baud_rate = baud_rate
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port

        self.log = logging.getLogger(name)
        self._ser = None
        self._ser_lock = threading.Lock()
        self._client_sock = None
        self._client_lock = threading.Lock()
        self._stop = threading.Event()

    # -- lifecycle -------------------------------------------------------

    def start(self):
        threading.Thread(target=self._serialLoop, name=f'{self.name}-serial', daemon=True).start()
        threading.Thread(target=self._tcpAcceptLoop, name=f'{self.name}-tcp', daemon=True).start()

    def stop(self):
        self._stop.set()
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
        self._closeClient(reason="bridge shutting down")

    # -- serial side: Mega -> TCP client -----------------------------------

    def _openSerial(self):
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
                self.log.info(f"Opened {self.serial_port_name} @ {self.baud_rate}")
                return ser
            except serial.SerialException as e:
                self.log.warning(
                    f"Could not open {self.serial_port_name} ({e}) -- retrying in {SERIAL_RETRY_SECONDS}s"
                )
                time.sleep(SERIAL_RETRY_SECONDS)
        return None

    def _serialLoop(self):
        while not self._stop.is_set():
            ser = self._openSerial()
            if ser is None:
                return  # stop() was called while waiting to open

            with self._ser_lock:
                self._ser = ser
            self._forwardToClient(b"BRIDGE SERIAL_UP\n")

            try:
                while not self._stop.is_set():
                    line = ser.readline()  # blocks up to `timeout` seconds
                    if not line:
                        continue  # just a read timeout, keep looping
                    self.log.info(f"SERIAL -> TCP: {line!r}")
                    self._forwardToClient(line)
            except (serial.SerialException, OSError) as e:
                self.log.warning(f"Serial link to {self.serial_port_name} dropped ({e}) -- reopening")
                self._forwardToClient(b"BRIDGE SERIAL_DOWN\n")
            finally:
                with self._ser_lock:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    self._ser = None
            # loop back around and try to reopen, unless stopping

    def _forwardToClient(self, line_bytes):
        with self._client_lock:
            sock = self._client_sock
        if sock is None:
            self.log.info("SERIAL -> TCP: dropped, no client connected")
            return  # no main PC connected right now -- drop it (monitoring only)
        try:
            sock.sendall(line_bytes)
            self.log.info("SERIAL -> TCP: sent OK")
        except OSError as e:
            self.log.warning(f"Failed sending to TCP client ({e}) -- dropping connection")
            self._closeClient()

    # -- TCP side: TCP client -> Mega ---------------------------------------

    def _tcpAcceptLoop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.tcp_host, self.tcp_port))
        server.listen(1)
        self.log.info(f"Listening for main PC on {self.tcp_host}:{self.tcp_port}")

        server.settimeout(1.0)
        while not self._stop.is_set():
            try:
                sock, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break  # server socket closed during stop()

            self.log.info(f"Main PC connected from {addr}")
            self._closeClient(reason="replaced by new connection")  # only one client at a time
            with self._client_lock:
                self._client_sock = sock

            with self._ser_lock:
                serial_up = self._ser is not None
            self._forwardToClient(b"BRIDGE CONNECTED SERIAL_UP\n" if serial_up else b"BRIDGE CONNECTED SERIAL_DOWN\n")

            self._clientRecvLoop(sock)

        server.close()

    def _clientRecvLoop(self, sock):
        buffer = b''
        sock.settimeout(1.0)
        while not self._stop.is_set():
            try:
                chunk = sock.recv(TCP_RECV_BUFSIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break  # client closed the connection

            buffer += chunk
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                self.log.info(f"TCP -> SERIAL: {line!r}")
                self._forwardToSerial(line + b'\n')

        self.log.info("Main PC disconnected")
        self._closeClient()

    def _forwardToSerial(self, line_bytes):
        with self._ser_lock:
            ser = self._ser
        if ser is None:
            self.log.warning(f"Dropping {line_bytes.strip()!r} -- {self.serial_port_name} not open")
            return
        try:
            ser.write(line_bytes)
            ser.flush()
            self.log.info(f"TCP -> SERIAL: wrote {len(line_bytes)} bytes OK")
        except (serial.SerialException, OSError) as e:
            self.log.warning(f"Failed writing to {self.serial_port_name} ({e})")

    def _closeClient(self, reason=None):
        """
        reason=None: plain close, no message -- used when the client is
        already gone (they disconnected themselves) or the socket is
        already broken, so there's no one to tell.
        reason=<str>: best-effort notice sent before closing, for cases
        where THIS bridge is the one deciding to end a still-live
        connection (e.g. a new client displacing this one).
        """
        with self._client_lock:
            if self._client_sock is not None:
                if reason:
                    try:
                        self._client_sock.sendall(f"BRIDGE DISCONNECTING {reason}\n".encode())
                    except OSError:
                        pass
                try:
                    self._client_sock.close()
                except Exception:
                    pass
                self._client_sock = None


def main():
    bridges = [
        MegaBridge(side['name'], side['serial_port'], BAUD_RATE, TCP_HOST, side['tcp_port'])
        for side in SIDES
    ]

    for bridge in bridges:
        bridge.start()

    logging.info("Bridge running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Shutting down...")
        for bridge in bridges:
            bridge.stop()


if __name__ == '__main__':
    main()
