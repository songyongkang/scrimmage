#!/usr/bin/env sh

# Setup environment variables
SCRIMMAGE_ROOT=/opt/scrimmage
export PATH=${SCRIMMAGE_ROOT}/bin:${PATH}
export JSBSIM_ROOT=${SCRIMMAGE_ROOT}/etc/JSBSim
export LD_LIBRARY_PATH=${SCRIMMAGE_ROOT}/lib:${LD_LIBRARY_PATH}
export CMAKE_PREFIX_PATH=${SCRIMMAGE_ROOT}:${CMAKE_PREFIX_PATH}

# Create user's ~/.scrimmage directory if it doesn't exist.
if [ ! -e ${HOME}/.scrimmage ]; then
    mkdir -p ${HOME}/.scrimmage/env
fi

# Create user's local setup.bash script if it doesn't exist.
USER_SETUP_SCRIPT=${HOME}/.scrimmage/setup.bash
if [ ! -e ${USER_SETUP_SCRIPT} ]; then
    touch ${USER_SETUP_SCRIPT}
fi

# If this script isn't sourced in the user's ~/.scrimmage/setup.bash script,
# add it. We allow the user to comment out the source.
LINE="source ${SCRIMMAGE_ROOT}/setup.sh"
grep "${LINE}" "${USER_SETUP_SCRIPT}" > /dev/null 2>&1
if [ $? != 0 ]; then
    echo ${LINE} >> ${USER_SETUP_SCRIPT}
fi
