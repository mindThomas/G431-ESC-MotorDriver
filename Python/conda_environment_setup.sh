#!/bin/bash
if [ -z ${CONDA_PREFIX} ]; then 
    echo "Conda environment has to be activated before running this script"
    return
fi

REPO_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd $CONDA_PREFIX
conda deactivate
mkdir -p ./etc/conda/activate.d
mkdir -p ./etc/conda/deactivate.d
touch ./etc/conda/activate.d/env_vars.sh
touch ./etc/conda/deactivate.d/env_vars.sh
cat >> ./etc/conda/activate.d/env_vars.sh <<EOF
#!/bin/sh
# Absolute path this script is in
SCRIPTPATH="\$( cd "\$( dirname "\${BASH_SOURCE[0]}" )" && pwd )"
echo \$SCRIPTPATH

export OLD_PATH=\$PATH
export OLD_PYTHONPATH=\$PYTHONPATH
export OLD_LD_LIBRARY_PATH=\$LD_LIBRARY_PATH

export ACADOS_SOURCE_DIR="~/repos/acados"

export PATH=\$PATH # add binaries needed within the environment here
export PYTHONPATH=$REPO_PATH:\$PYTHONPATH # add libraries needed by Python here
export LD_LIBRARY_PATH=\$SCRIPTPATH/../../../lib:\$ACADOS_SOURCE_DIR/lib:\$LD_LIBRARY_PATH # add libraries needed within the environment here
EOF
cat >> ./etc/conda/deactivate.d/env_vars.sh <<EOF
#!/bin/sh
export PATH=\$OLD_PATH
export PYTHONPATH=\$OLD_PYTHONPATH
export LD_LIBRARY_PATH=\$OLD_LD_LIBRARY_PATH
EOF
cd -