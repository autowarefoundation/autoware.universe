#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)

# Prevent root execution
if [ $(id -u) -eq 0 ]; then
  echo "Don't run as root" >&2
  exit 1
fi

read -p ">  Do you run setup? This will take a while. [y/N] " answer

if [[ $answer = [cC] ]]; then
  if !(command -v cowsay  > /dev/null 2>&1); then
    sudo apt install -y cowsay
  fi

  export answer=y
fi

case $answer in
  [yY]* )
    if !(command -v ansible-playbook  > /dev/null 2>&1); then
      sudo apt install -y ansible
    fi

    cd $SCRIPT_DIR/ansible
    ansible-playbook -i localhost, $SCRIPT_DIR/ansible/localhost-setup-ubuntu20.04-devpc.yml -i $SCRIPT_DIR/inventories/local-dev.ini -e AUTOWARE_DIR=$SCRIPT_DIR --ask-become-pass
    echo -e "\e[32mComplete \e[0m"
    ;;
  * )
    echo -e "\e[33mCanceled \e[0m"
    ;;
esac
