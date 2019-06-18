#!/usr/bin/env bash

function ok (){
  if [ $? -eq 0 ]; then
     echo -e "\e[92m-----> OK, going on!\e[0m"
  else
     echo -e "\e[31m-----> Failure, exiting install script, noooo :|\e[0m"
     exit $?
  fi
}

echo -e "\e[92m#### Downloading Model ####\e[0m"
mkdir -p model
cd model
if [ ! -e "deepspeech-0.4.1-models.tar.gz" ]
then
    wget --no-check-certificate --tries=3 https://github.com/mozilla/DeepSpeech/releases/download/v0.4.1/deepspeech-0.4.1-models.tar.gz
fi
tar xvfz deepspeech-0.4.1-models.tar.gz
ok