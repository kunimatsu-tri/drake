#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on macOS.

set -euxo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

if command -v conda &>/dev/null; then
  echo 'WARNING: Anaconda is NOT supported for building and using the Drake Python bindings' >&2
fi

if ! command -v brew &>/dev/null; then
  /bin/bash -c \
    "$(/usr/bin/curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

# Do not automatically update before running brew install, brew upgrade, or brew
# tap during execution of this script. We manually update where necessary and
# retry if the update fails.
export HOMEBREW_NO_AUTO_UPDATE=1

# Do not automatically cleanup installed, upgraded, and/or reinstalled formulae
# during execution of this script. Manually run brew cleanup after the script
# completes or wait for the next automatic cleanup if necessary
export HOMEBREW_NO_INSTALL_CLEANUP=1

brew update

# TODO(jamiesnape): Remove lines tapping robotlocomotion/director and
# uninstalling ipopt@3.11, vtk@8.2, ospray@1.8, and embree@3.5 on or after
# 2021-03-01.
brew tap robotlocomotion/director
brew uninstall --force $(cat <<EOF
robotlocomotion/director/ipopt@3.11
robotlocomotion/director/vtk@8.2
robotlocomotion/director/ospray@1.8
robotlocomotion/director/embree@3.5
EOF
)

# Ensure numpy is updated to the most recent version to avoid conflicts with
# robotlocomotion/drake/numpy@1.19.4
# TODO(jamiesnape): Remove line upgrading numpy or after 2021-02-01.
brew list numpy &>/dev/null \
  && (brew outdated numpy >/dev/null || brew upgrade numpy)

brew bundle --file="${BASH_SOURCE%/*}/Brewfile" --no-lock

if ! command -v /usr/local/opt/python@3.8/bin/pip3 &>/dev/null; then
  echo 'ERROR: pip3 for python@3.8 is NOT installed. The post-install step for the python@3.8 formula may have failed.' >&2
  exit 2
fi

/usr/local/opt/python@3.8/bin/pip3 install --upgrade --requirement "${BASH_SOURCE%/*}/requirements.txt"
