import os
from datetime import datetime
from rospkg import RosPack

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

def find_package_path(package_name):
    rospack = RosPack()
    return rospack.get_path(package_name)

def find_file(package_name, file_name):
    package_path = find_package_path(package_name)
    return os.path.join(package_path, file_name)