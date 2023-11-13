import tornado.ioloop
import tornado.web
import tornado.websocket
import json
import os
# export SERVER_API_KEY='xxxx'
API_KEY = os.getenv('SERVER_API_KEY')

# Define the value to be shared
shared_value = ""

class APIKeyMixin:
    def check_api_key(self):
        return self.request.headers.get("Authorization") == f"Bearer {API_KEY}"
    
class SetValueHandler(APIKeyMixin, tornado.web.RequestHandler):
    def get(self):
        if not self.check_api_key():
            self.set_status(401)
            self.write("Unauthorized: Invalid API key")
            return
        global shared_value
        value = self.get_argument("value", None)
        if value:
            shared_value = value
            self.write(f"Value set to: {shared_value}")
            # Broadcast to WebSocket clients
            for client in WebSocketHandler.clients:
                client.write_message(json.dumps({"value": shared_value}))
        else:
            self.set_status(400)
            self.write("Bad Request: 'value' parameter is missing")

class GetValueHandler(APIKeyMixin, tornado.web.RequestHandler):
    def get(self):
        if not self.check_api_key():
            self.set_status(401)
            self.write("Unauthorized: Invalid API key")
            return
        self.write(f"Current Value: {shared_value}")

class WebSocketHandler(tornado.websocket.WebSocketHandler):
    clients = set()

    def open(self):
        # Extract API key from query parameters
        api_key = self.get_argument("apiKey", None)
        if api_key != API_KEY:
            self.close(code=4001, reason="Invalid API key")  # Close the connection
            return
        WebSocketHandler.clients.add(self)

    def on_close(self):
        WebSocketHandler.clients.remove(self)

def make_app():
    return tornado.web.Application([
        (r"/setvalue", SetValueHandler),
        (r"/getvalue", GetValueHandler),
        (r"/websocket", WebSocketHandler),
    ])

if __name__ == "__main__":
    app = make_app()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()
