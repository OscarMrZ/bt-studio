import json
import subprocess
from websocket_server import WebsocketServer
import os
import uuid
import base64
import zipfile
import psutil
import threading

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

def prepare_launch_files(launch_files, project_name):

    binary_path = "/workspace/binaries/" + project_name + ".zip"
    project_path = "/workspace/worlds/" + project_name
    os.mkdir(project_path)

    if launch_files is not None:
        try:
            # Convert base64 to binary
            binary_content = base64.b64decode(launch_files)

            # Save the binary content as a file
            with open(binary_path, 'wb') as file:
                file.write(binary_content)

            # Unzip the file
            with zipfile.ZipFile(binary_path, 'r') as zip_ref:
                zip_ref.extractall(project_path)

        except Exception as e:
            file.write(
                "An error occurred while opening zip_path as r:" + str(e))

def launch_gzserver_and_client(launch_path, gzclient_cmd):

    try:
        subprocess.run(["ros2", "launch", launch_path], check=True)
        subprocess.run(gzclient_cmd, shell=True, check=True)
        print("Gazebo server and client launched in VNC session.")

    except subprocess.CalledProcessError as e:
        print(f"An error occurred while launching Gazebo: {e}")

def launch_simulation(project_name):

    # Configuration of gz_client
    display = ":2"
    dri_path = get_dri_path()
    gzclient_cmd = f"DISPLAY={display} VGL_DISPLAY={dri_path} vglrun gzclient --verbose"

    # Configuration of gazebo
    launch_path = "/workspace/worlds/" + project_name + "/launcher.py"

    # Start the simulation in a separate thread
    simulation_thread = threading.Thread(target=launch_gzserver_and_client, args=(launch_path, gzclient_cmd))
    simulation_thread.start()

def stop_running_simulations():

    for process in psutil.process_iter(['pid', 'name']):

        # Check if the process is a Gazebo related process
        if 'gzserver' in process.info['name'] or 'gzclient' in process.info['name']:
            print(f"Stopping {process.info['name']} with PID {process.info['pid']}")
            psutil.Process(process.info['pid']).terminate()

def message_received(client, server, message):

    # print(f"Message received: {message}")
    
    try:
        data = json.loads(message)  

        if data.get("command") == "launch":
            
            # Extract the config
            launch_config = data.get("data")
            launch_files = launch_config["launch_files"]
            project_name = launch_config["project_name"]

            # Prepare the launch files received in the message
            prepare_launch_files(launch_files, project_name)

            # Stop all the running simulations
            # stop_running_simulations()

            # Launch the gazebo simulation
            launch_simulation(project_name)
            print("potato??")

            # Signal the client that the simulation is ready
            ready_msg = {
                "id": data.get("id"),
                "command": "state-changed",
                "data": {
                    "state": "ready"
                }
            }   
            server.send_message(client, json.dumps(ready_msg))

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

            # Send an ack to the connect message
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
