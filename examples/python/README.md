# Install environment for network application development

This document describes the installation procedure of the environment for network application development

1. Install Python (recommended version is 3.8, but 3.9 and 3.10 are supported as well)
    1. https://linux.how2shout.com/install-python-3-9-or-3-8-on-ubuntu-22-04-lts-jammy-jellyfish/
2. Install pip and git
    1. sudo apt install python3-pip git
3. Create and source the virtual environment (optional)
    1. python -m venv env
    2. source env/bin/activate
4. Clone the Reference-NetApp repository
    1. git clone https://github.com/5G-ERA/Reference-NetApp.git
5. Install the requirements from the era_5g_network_application_template package
    1. cd Reference-NetApp/src/python/era_5g_network_application_template/
    2. pip install -r requirement.txt
6. Install the era-5g-client package
    1. pip install era-5g-client

This is the minimal setup needed for developing and running the network application. For deploying the network application to the middleware, the created network application must be converted into a docker image and uploaded to the docker repository. The docker must be installed in the environment to build the images: https://docs.docker.com/engine/install/
