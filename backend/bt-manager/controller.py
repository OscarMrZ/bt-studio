import json
import subprocess
from websocket_server import WebsocketServer
import os
import uuid

def get_dri_path():

    directory_path = '/dev/dri'
    dri_path = ""
    if os.path.exists(directory_path) and os.path.isdir(directory_path):
        files = os.listdir(directory_path)
        if ("card1" in files):
            dri_path = os.path.join(
                "/dev/dri", os.environ.get("DRI_NAME", "card1"))
        else:
            dri_path = os.path.join(
                "/dev/dri", os.environ.get("DRI_NAME", "card0"))
    
    return dri_path

def new_client(client, server):

    print(f"New client connected and was given id {client['id']}")



def message_received(client, server, message):

    print(f"Message received: {message}")
    
    try:
        data = json.loads(message)
        if data.get("command") == "hiper":
            display = ":2"  # Example DISPLAY value, adjust as per your setup
            dri_path = get_dri_path()
            gzserver_cmd = "gzserver"  # Basic command to start gzserver
            gzclient_cmd = f"DISPLAY={display} VGL_DISPLAY={dri_path} vglrun gzclient --verbose"

            # Execute the gzserver and gzclient commands
            subprocess.run(["ros2", "launch", "/workspace/worlds/simulation.launch.py"], check=True)
            subprocess.run(gzclient_cmd, shell=True, check=True)
            print("Gazebo server and client launched in VNC session.")

        elif data.get("command") == "launch":

            print(message)

        elif data.get("command") == "connect":

            # Send an acknowledgment message with a unique id, command "state-changed", and state "connected"
            ack_message = {
                "id": str(uuid.uuid4()),
                "command": "state-changed",
                "data": {
                    "state": "connected"
                    }
                }
                    
            server.send_message(client, json.dumps(ack_message))

                # Send an ack message with a unique id
            ack_message = {
                "id": data.get("id"),
                "command": "ack",
                "data": None
            }
            server.send_message(client, json.dumps(ack_message))

    except json.JSONDecodeError:
        print("Received non-JSON message")
    except subprocess.CalledProcessError as e:
        print(f"Failed to launch Gazebo: {e}")

def client_left(client, server):
    print(f"Client({client['id']}) disconnected")

server = WebsocketServer(port=1905, host='0.0.0.0')
server.set_fn_new_client(new_client)
server.set_fn_message_received(message_received)
server.set_fn_client_left(client_left)
server.run_forever()
