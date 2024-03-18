#! /usr/bin/env bash

## install the python-bloom fakeroot

function install_pkg()
{
  pkg_found=$(dpkg -l | grep ${1})
  if [[ $? -eq 0 ]]
  then
    echo "${1} already installed."
  else
    echo "Install ${1} ..."
    sudo apt-get install ${1} -y
  fi
}

install_pkg python-bloom
install_pkg fakeroot

echo -e "\n\033[1;32m ~~ (1). Delete old debian folders in the directory...\033[0m"
rm -rf debian/ obj-x86_64-linux-gnu/

echo -e "\n\033[1;32m ~~ (2). Delete any backup files...\033[0m"
find . -type f -name '*~' -delete

echo -e "\n\033[1;32m ~~ (3). Create debian packages...\033[0m\n"
bloom-generate rosdebian --os-name ubuntu --os-version `echo $(lsb_release -sc)` --ros-distro `echo ${ROS_DISTRO}`

# sed 's#dh_shlibdeps*#dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info#g' debian/rules
##  when dpkg-shlibdeps: error: no dependency information found for
## adding bellow code to debian/rules
##
## override_dh_shlibdeps:
## dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info
## It is necessary to insert a TAB instead of spaces before "dh_shlibdeps ..."

target_string=$(grep "dh_shlibdeps " debian/rules)
replace_string="  dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info"
sed  -i "s#${target_string}#${replace_string}#g" debian/rules

fakeroot debian/rules binary

echo -e "\n\033[1;32m ~~ (4). Delete old debian folders in the directory...\033[0m"
rm -rf debian/ obj-x86_64-linux-gnu/

echo -e "\n\033[1;32m ~~ (5). Delete any backup files...\033[0m"
find . -type f -name '*~' -delete
