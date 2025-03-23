#! /bin/bash

if [ -z ${TC_DOCKER_REPO+x} ]; then
	echo "guessing TC_DOCKER_REPO based on input";
	if [[ $@ =~ .*tc_fmu.* ]]; then
		# nuttx-tcfmu-v{1,2,3,4,5}
		TC_DOCKER_REPO="tcio/tc-dev-nuttx-focal:2022-08-12"
	elif [[ $@ =~ .*navio2.* ]] || [[ $@ =~ .*raspberry.* ]] || [[ $@ =~ .*beaglebone.* ]] || [[ $@ =~ .*pilotpi.default ]]; then
		# beaglebone_blue_default, emlid_navio2_default, tc_raspberrypi_default, scumaker_pilotpi_default
		TC_DOCKER_REPO="tcio/tc-dev-armhf:2023-06-26"
	elif [[ $@ =~ .*pilotpi.arm64 ]]; then
		# scumaker_pilotpi_arm64
		TC_DOCKER_REPO="tcio/tc-dev-aarch64:2022-08-12"
	elif [[ $@ =~ .*navio2.* ]] || [[ $@ =~ .*raspberry.* ]] || [[ $@ =~ .*bebop.* ]]; then
		# posix_rpi_cross, posix_bebop_default
		TC_DOCKER_REPO="tcio/tc-dev-armhf:2023-06-26"
	elif [[ $@ =~ .*clang.* ]] || [[ $@ =~ .*scan-build.* ]]; then
		# clang tools
		TC_DOCKER_REPO="tcio/tc-dev-clang:2021-02-04"
	elif [[ $@ =~ .*tests* ]]; then
		# run all tests with simulation
		TC_DOCKER_REPO="tcio/tc-dev-simulation-bionic:2021-12-11"
	fi
else
	echo "TC_DOCKER_REPO is set to '$TC_DOCKER_REPO'";
fi

# otherwise default to nuttx
if [ -z ${TC_DOCKER_REPO+x} ]; then
	TC_DOCKER_REPO="tcio/tc-dev-nuttx-focal:2022-08-12"
fi

# docker hygiene

#Delete all stopped containers (including data-only containers)
#docker rm $(docker ps -a -q)

#Delete all 'untagged/dangling' (<none>) images
#docker rmi $(docker images -q -f dangling=true)

echo "TC_DOCKER_REPO: $TC_DOCKER_REPO";

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

docker run -it --rm -w "${SRC_DIR}" \
	--env=AWS_ACCESS_KEY_ID \
	--env=AWS_SECRET_ACCESS_KEY \
	--env=BRANCH_NAME \
	--env=CCACHE_DIR="${CCACHE_DIR}" \
	--env=CI \
	--env=CODECOV_TOKEN \
	--env=COVERALLS_REPO_TOKEN \
	--env=LOCAL_USER_ID="$(id -u)" \
	--env=TC_ASAN \
	--env=TC_MSAN \
	--env=TC_TSAN \
	--env=TC_UBSAN \
	--env=TRAVIS_BRANCH \
	--env=TRAVIS_BUILD_ID \
	--publish 14556:14556/udp \
	--volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
	--volume=${SRC_DIR}:${SRC_DIR}:rw \
	${TC_DOCKER_REPO} /bin/bash -c "$1 $2 $3"
