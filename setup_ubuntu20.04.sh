#!/bin/bash

noninteractive=0
while getopts c OPT
do
  case $OPT in
    "c" ) noninteractive=1
      echo "run setup with a noninteractive mode"
      ;;
    "*" )
      ;;
  esac
done

# shellcheck disable=SC2046,SC2086,SC2164
SCRIPT_DIR=$(cd $(dirname $0); pwd)

# Prevent root execution
if [ $noninteractive -eq 0 ]; then
  if [ "$(id -u)" -eq 0 ]; then
    echo "Do not run setup as root!" >&2
    exit 1
  fi
fi

if [ $noninteractive -eq 1 ]; then
  answer="y"
else
  echo "Setting up the pre-build environment for AutowareArchitectureProposal.IV can take up to 45 minutes."
  read -rp ">  Are you sure you want to run setup? [y/N] " answer
fi

if [[ $answer = [cC] ]]; then
  if ! (command -v cowsay  > /dev/null 2>&1); then
    sudo apt install -y cowsay
  fi

  export answer=y
fi

case $answer in
  [yY]* )
    if ! (command -v ansible-playbook  > /dev/null 2>&1); then
      sudo apt install -y ansible
    fi

    cd "$SCRIPT_DIR/ansible" || exit
    if [ $noninteractive -eq 1 ]; then
      ansible-playbook -i localhost, "$SCRIPT_DIR/ansible/localhost-setup-ubuntu20.04-devpc.yml" -i "$SCRIPT_DIR/inventories/local-dev.ini -e AUTOWARE_DIR=$SCRIPT_DIR" --extra-vars "yn_gpu=y"
    else
      ansible-playbook -i localhost, "$SCRIPT_DIR/ansible/localhost-setup-ubuntu20.04-devpc.yml" -i "$SCRIPT_DIR/inventories/local-dev.ini -e AUTOWARE_DIR=$SCRIPT_DIR" --ask-become-pass
    fi

    # shellcheck disable=SC2181
    if [ "$?" = "0" ]; then
      echo -e "\e[32mComplete \e[0m"
      exit 0
    else
      echo -e "\e[31mFailed \e[0m"
      exit 1
    fi
    ;;
  * )
    echo -e "\e[33mCancelled \e[0m"
    exit 1
    ;;
esac
