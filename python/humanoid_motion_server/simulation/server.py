"""Own the Meshcat server in a launch-managed process, including shutdown."""

import argparse
import asyncio


def main():
    from meshcat.servers.zmqserver import ZMQWebSocketBridge
    parser = argparse.ArgumentParser()
    parser.add_argument('--zmq-url', required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7000)
    args = parser.parse_args()

    class BoundBridge(ZMQWebSocketBridge):
        def make_app(self):
            app = super().make_app()
            # In Meshcat 0.3.2 host only affects its advertised URL, not listen().
            listen = app.listen

            def bound_listen(port, **kwargs):
                self.http_server = listen(port, address=self.host, **kwargs)
                return self.http_server

            app.listen = bound_listen
            return app

    bridge = BoundBridge(zmq_url=args.zmq_url, host=args.host, port=args.port)
    print(f'Meshcat simulation: {bridge.web_url}', flush=True)
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        async def shutdown_http():
            bridge.http_server.stop()
            for websocket in tuple(bridge.websocket_pool):
                websocket.close()
                # Do not wait for a browser's closing handshake during launch shutdown.
                if websocket.ws_connection is not None:
                    websocket.ws_connection.stream.close()
            await bridge.http_server.close_all_connections()
            pending = [task for task in asyncio.all_tasks() if task is not asyncio.current_task()]
            for task in pending:
                task.cancel()
            if pending:
                await asyncio.gather(*pending, return_exceptions=True)

        bridge.ioloop.run_sync(shutdown_http, timeout=3)
        bridge.zmq_stream.close()
        bridge.zmq_socket.close(linger=0)
        bridge.ioloop.close(all_fds=True)
