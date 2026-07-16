#!/usr/bin/env python3
"""
test_client.py — minimal interactive TCP client for testing
mega_serial_bridge.py directly, without TouchDesigner.

Windows doesn't ship a telnet client by default (it's an optional feature),
so this is a more reliable cross-platform stand-in for "just telnet into
the port and type commands."

Usage:
    python test_client.py <host> <port>

Example (testing the LEFT Mega via the mini PC bridge, from any machine
on the same network as the mini PC):
    python test_client.py 192.168.1.50 9000

Type commands (e.g. STATUS, HOME 0, SETPOS 0 200) and press Enter.
Anything the Mega sends back is printed as it arrives on a background
thread, so you don't have to wait for a reply before typing the next
command. Ctrl+C to quit.
"""

import socket
import sys
import threading


def _recvLoop(sock):
    buffer = b''
    while True:
        try:
            chunk = sock.recv(4096)
        except OSError:
            break
        if not chunk:
            print("\n[connection closed by bridge]")
            break
        buffer += chunk
        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)
            print(f"< {line.decode(errors='replace').strip()}")


def main():
    if len(sys.argv) != 3:
        print(f"Usage: python {sys.argv[0]} <host> <port>")
        sys.exit(1)

    host, port = sys.argv[1], int(sys.argv[2])
    sock = socket.create_connection((host, port), timeout=5)
    print(f"Connected to {host}:{port}. Type commands, Ctrl+C to quit.")

    threading.Thread(target=_recvLoop, args=(sock,), daemon=True).start()

    try:
        while True:
            line = input('> ')
            if not line:
                continue
            sock.sendall((line + '\n').encode())
    except (KeyboardInterrupt, EOFError):
        print("\nClosing.")
    finally:
        sock.close()


if __name__ == '__main__':
    main()
