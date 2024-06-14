import json
import sys 

def load_qlog(file_path):
    with open(file_path, 'r') as file:
        qlog_data = json.load(file)
    return qlog_data

def extract_congestion_window(qlog_data):
    congestion_windows = []
    for event in qlog_data['traces'][0]['events']:
        if event[0] == "transport" and "congestion_window" in event[3]:
            congestion_windows.append(event[3]["congestion_window"])
    return congestion_windows

# Load the qlog file
qlog_file_path = sys.argv[1]
qlog_data = load_qlog(qlog_file_path)

# Extract congestion_window
congestion_windows = extract_congestion_window(qlog_data)

print(congestion_windows)

