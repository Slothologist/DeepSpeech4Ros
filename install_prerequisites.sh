
echo -e "\e[92m#### Installing the necessary requirements for Bazel and Tensorflow! ####\e[0m"
start=

echo -e "\e[92m#### Bazel ####\e[0m"
sudo apt install -y build-essential openjdk-8-jdk python zip unzip

echo -e "\e[92m#### Tensorflow and Deepspeech ####\e[0m"
sudo apt install -y python3-numpy python3-dev python3-pip python3-wheel sox libsox-dev

duration=$((SECONDS-start))
echo -e "\e[92m#### Finished in" $duration "seconds ! You're good to go! ####\e[0m"
