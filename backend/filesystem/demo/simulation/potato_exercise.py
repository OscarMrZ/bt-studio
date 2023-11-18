import os

print("HAIL THE POTATO KING")
home_dir = os.path.expanduser('~')
f = open(f"{home_dir}/ws_gui.log", "w")
f.write("websocket_gui=ready")
f.close()

f = open(f"{home_dir}/ws_code.log", "w")
f.write("websocket_code=ready")
f.close()