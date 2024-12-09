
1. dependencies
'''
## Ros2
sudo apt-get install python3-vcstool
vcs import ros2_ws/src/ < package.repos 

## Python
python3.9 -m venv my_env
source my_env/bin/activate
pip3 isntall -r requirements.txt
'''