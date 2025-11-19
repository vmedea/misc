#!/usr/bin/python3
'''
Route telemetry received from liftoff to interested clients.
'''
# Mara Huldra 2025
# SPDX-License-Identifier: MIT
import asyncio
import socket

CMD_IN = ('0.0.0.0', 9003)
TEL_IN = ('0.0.0.0', 9001)

class Opcode:
    REGISTER = 0x00
    UNREGISTER = 0x01
    QUIT = 0xff

class CommandServerProtocol:
    def __init__(self, on_quit):
        self.clients = {}
        self.on_quit = on_quit

    def connection_made(self, transport):
        self.transport = transport
        sock = self.transport.get_extra_info("socket")
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def datagram_received(self, data, addr):
        print('Command: Received %r from %s' % (data.hex(), addr))
        if len(data) < 1:
            return
        op = data[0]
        if op == Opcode.REGISTER: # register/keepalive
            self.add_client(addr)
        elif op == Opcode.UNREGISTER:
            self.remove_client(addr)
        elif op == Opcode.QUIT:
            self.on_quit.set_result(True)

    def add_client(self, addr):
        # "established-over-unconnected technique"
        # so that we get an exception if the port is closed
        # and the remote host sends an ICMP error
        txsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        txsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        txsock.bind(CMD_IN)
        txsock.connect(addr)
        self.clients[addr] = txsock
        print(f'Clients: {list(self.clients.keys())}')

    def remove_client(self, addr):
        try:
            del self.clients[addr]
        except ValueError:
            pass
        print(f'Clients: {list(self.clients.keys())}')

    def broadcast(self, data):
        remove_clients = []
        for client, txsock in self.clients.items():
            try:
                txsock.send(data)
            except IOError as e: # remove clients that raise error
                print(f'Client error: {client} {e}')
                remove_clients.append(client)

        for client in remove_clients:
            self.remove_client(client)

class TelemetryServerProtocol:
    def __init__(self, command_server):
        self.command_server = command_server

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        # print('Telemetry: Received %r from %s' % (data.hex(), addr))
        self.command_server.broadcast(data)

async def main():
    print("Starting telemetry router")

    loop = asyncio.get_running_loop()
    on_quit = loop.create_future()

    transport_cmd, protocol_cmd = await loop.create_datagram_endpoint(
        lambda: CommandServerProtocol(on_quit),
        local_addr=CMD_IN)
    transport_tel, protocol_tel = await loop.create_datagram_endpoint(
        lambda: TelemetryServerProtocol(protocol_cmd),
        local_addr=TEL_IN)

    try:
        await on_quit
    finally:
        transport_tel.close()
        transport_cmd.close()

asyncio.run(main())
