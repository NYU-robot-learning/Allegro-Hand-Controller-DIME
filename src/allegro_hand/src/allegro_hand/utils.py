import os
from datetime import datetime

# Function to check and create a directory if not found
def make_dir(folder):
    if not os.path.exists(folder):
        print('Making Directory: {}'.format(folder))
        os.makedirs(folder)
    else:
        print('Directory Exists : {}'.format(folder))

# Function to get current date and time
def get_datetime():
    now = datetime.now()
    return now.strftime('%Y-%m-%d-%H-%M-%S-%f')