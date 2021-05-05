## Environment setup instructions
1. Install Anaconda or Miniconda
2. Create the environment `conda create -y --name esc python=3.7`
     1. Alternatively based on the environment file
`conda env create environment.yml -n esc`
3. Activate the environment
`conda activate esc`
4. Install libraries from the requirements file (can be omitted if the environment-file based setup is used: `pip3 install -r requirements.txt`
    4.1 Alternatively `conda install --force-reinstall -y -q --name esc -c conda-forge --file requirements.txt`
5. Configure the environment to use the libraries (setting path variables etc.). It is important to run this command from the same directory as where it lives.
`source conda_environment_setup.sh`


## Notes

### Usage with PyCharm
When using this with PyCharm you need to launch PyCharm from a console after activating the environment. Furthermore you should point the Interpreter settings to the conda enviroment within PyCharm.