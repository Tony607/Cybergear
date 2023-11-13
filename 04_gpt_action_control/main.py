import websocket
import threading
import json
import os
from robo_replay import RoboReplay
import logging
import os
API_KEY = os.getenv('SERVER_API_KEY')
robo = RoboReplay()

def on_message(ws, message):
    logging.info(f"Received message: {message}")
    value = json.loads(message).get("value")
    logging.info(value)
    # Construct the path to the file in the 'records' subfolder
    file_path = os.path.join(os.path.dirname(__file__), "records", f"positions-{value}.csv")
    if os.path.exists(file_path):
        robo.enable()
        robo.replay(file_path)
        robo.disable()
    else:
        logging.info(f"No record file found: {file_path}")
        

def on_error(ws, error):
    logging.info(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    logging.info("### Connection closed ###")

def on_open(ws):
    logging.info("Connected to the WebSocket server")

if __name__ == "__main__":
    websocket.enableTrace(True)
    # Load the URL from schema.json
    with open("schema.json", "r") as file:
        data = json.load(file)
        # Extract the base URL and remove the 'https://' prefix
        base_url = data["servers"][0]["url"]
        if base_url.startswith("https://"):
            base_url = base_url[8:]  # remove 'https://'

    # Construct the WebSocket URL using the base URL from schema.json
    ws_url = f"ws://{base_url}/websocket?apiKey={API_KEY}"
    logging.info(ws_url)
    ws = websocket.WebSocketApp(ws_url,
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    wst = threading.Thread(target=ws.run_forever)
    wst.start()

    input("Press enter to quit\n\n")
    ws.close()
