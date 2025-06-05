import json
import subprocess

# Load the JSON data
with open('bags.json') as f:
    data = json.load(f)

# Path to your bash script
bash_script = './launch_rosScript.sh'  # Replace with your actual bash script path

# Iterate through each experiment
for experiment in data['experiments']:
    name = experiment['name']
    bag_file = experiment['bag_file']
    ground_truth = experiment['ground_truth']
    
    # Call the bash script with the three arguments
    try:
        subprocess.run([bash_script, name, bag_file, ground_truth], check=True)
        print(f"Successfully processed experiment: {name}")
    except subprocess.CalledProcessError as e:
        print(f"Error processing experiment {name}: {e}")