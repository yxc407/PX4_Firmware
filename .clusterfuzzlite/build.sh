#!/usr/bin/env bash -eu

TC_FUZZ=1 make tc_sitl
cp build/tc_sitl_default/bin/tc $OUT/tc
