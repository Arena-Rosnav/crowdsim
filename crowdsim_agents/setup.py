## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

CROWDSIM_AGENTS = "crowdsim_agents"

def crowdsim_forces(prefix):
    CROWDSIM_FORCES = f"${prefix}.crowdsim_forces"

    packages = [CROWDSIM_FORCES]
    
    # for pkg in next(os.walk(os.path.join("src","crowdsim_agents","forces","forcemodels")))[1]:
    #     packages.append(f"{PEDSIM_FORCES}.forcemodels.{pkg}") 

    return packages

def crowdsim_semantic(prefix):
    PEDSIM_SEMANTIC = f"${prefix}.crowdsim_semantic"

    packages = [PEDSIM_SEMANTIC]

    return packages

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        CROWDSIM_AGENTS,
        *crowdsim_forces(CROWDSIM_AGENTS),
        *crowdsim_semantic(CROWDSIM_AGENTS)
    ],
    package_dir={'': 'src',}
)

setup(**setup_args)