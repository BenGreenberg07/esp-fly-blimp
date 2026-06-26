"""
Make cflib's UDP driver compatible with ESP-Drone / ESP-FLY framing.

ESP-Drone wraps every CRTP packet sent over UDP with a trailing checksum byte:

    [ CRTP header ][ data bytes... ][ cksum ]      cksum = sum(all prior bytes) & 0xFF

Stock cflib (0.1.32) does NOT add this byte on send, and does NOT strip it on
receive, so the link connects but the drone rejects every packet
("udp packet cksum unmatched") and nothing works.

Importing this module monkey-patches cflib in place to add/strip the checksum.
Import it BEFORE cflib.crtp.init_drivers().
"""

import socket
import struct

from cflib.crtp import udpdriver
from cflib.crtp.crtpstack import CRTPPacket


def _send_packet(self, pk):
    if self.socket is None:
        return
    try:
        raw = (pk.header,) + struct.unpack("B" * len(pk.data), pk.data)
        cksum = sum(raw) & 0xFF                      # ESP-Drone checksum
        raw = raw + (cksum,)
        self.socket.send(struct.pack("B" * len(raw), *raw))
    except Exception as e:
        if self.link_error_callback:
            self.link_error_callback(
                "UdpDriver: Could not send packet to Crazyflie\nException: %s" % e)


def _run(self):
    self._socket.settimeout(1.0)
    while True:
        if self._sp:
            break
        try:
            packet = self._socket.recv(1024)
            data = struct.unpack("B" * len(packet), packet)
            if len(data) >= 2:                        # need header + checksum
                crtp = data[:-1]                      # drop trailing checksum byte
                pk = CRTPPacket(header=crtp[0], data=crtp[1:])
                self._in_queue.put(pk)
        except socket.timeout:
            pass
        except Exception as e:
            import traceback
            if self._link_error_callback:
                self._link_error_callback(
                    "Error communicating with the Crazyflie\n"
                    "Exception:%s\n\n%s" % (e, traceback.format_exc()))


udpdriver.UdpDriver.send_packet = _send_packet
udpdriver._UdpReceiveThread.run = _run
