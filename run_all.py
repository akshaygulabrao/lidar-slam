import json
import subprocess
from pathlib import Path
import os 

# Load the JSON data
with open('bags.json') as f:
    data = json.load(f)

# Path to your bash script
bash_script = './launch_rosScript.sh'

root_dir = Path("/Volumes/hd1/NewerCollegeDataset")
gt_dir = Path(f"{root_dir}/collection 1 - newer college")
# Iterate through each experiment
for experiment in data['experiments']:
    name = experiment['name']
    bag_file = f"{root_dir}/{experiment['bag_file']}"
    ground_truth = f"{gt_dir}/{experiment['ground_truth']}"

    files = [bag_file,ground_truth]
    for file in files:
        if not os.path.isfile(file):
            print(file)
            raise FileNotFoundError

    try:
        subprocess.run([bash_script, name, bag_file, ground_truth], check=True)
        print(f"Successfully processed experiment: {name}")
    except subprocess.CalledProcessError as e:
        print(f"Error processing experiment {name}: {e}")