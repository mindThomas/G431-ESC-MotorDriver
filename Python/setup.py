import setuptools

with open('requirements.txt', 'r') as file:
    requirements = [line.strip('\n') for line in file.readlines() if line != '' and line != '\n']

setuptools.setup(
    name='g431_esc_motordriver',
    version='0.0.1',
    description='Python tools for the G431-ESC-MotorDriver project',
    author='Thomas Jespersen',
    author_email='thomasj@tkjelectronics.dk',
    #package_dir={"": "src"},
    #packages=find_packages(where="src"),
    packages=[package for package in setuptools.find_packages()],
    install_requires=requirements,    
)