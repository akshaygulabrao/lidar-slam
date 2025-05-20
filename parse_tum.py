import pandas as pd
import os
from pathlib import Path

def convert_to_tum(input_file):
    """
    Convert the file to proper TUM format, handling the # comment header.
    Input format: #sec,nsec,x,y,z,qx,qy,qz,qw
    Output format: timestamp(sec.nsec) x y z qx qy qz qw
    """
    filename = os.path.basename(input_file)[:-3] + "tum"
    df = pd.read_csv(input_file, comment='#', header=None, 
                    names=['sec', 'nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    df['timestamp'] = df['sec'] + df['nsec'] * 1e-9
    tum_df = df[['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']]
    tum_df.to_csv(filename, sep=' ', header=False, index=False, float_format='%.9f')

input_filename = '/Users/ox/workspace/newer-college-1/collection1-newer-college/ground_truth/gt-nc-quad-easy.csv'
convert_to_tum(input_filename)