#!/usr/bin/env bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
DIR=$(pwd)

function ok (){
  if [ $? -eq 0 ]; then
     echo -e "\e[92m-----> OK, going on!\e[0m"
  else
     echo -e "\e[31m-----> Failure, exiting install script, noooo :|\e[0m"
     exit $?
  fi
}

export MAIN=$DIR/deepspeech_install
CPU_CORES='nproc --all'
start=$SECONDS



echo -e "\e[92m#### Downloading Model ####\e[0m"
mkdir -p $MAIN/model
cd $MAIN/model
if [ ! -e "deepspeech-0.2.0-models.tar.gz" ]
then
    wget https://github.com/mozilla/DeepSpeech/releases/download/v0.2.0/deepspeech-0.2.0-models.tar.gz
fi
tar xvfz deepspeech-0.2.0-models.tar.gz


echo -e "\e[92m#### Bazel ####\e[0m"
cd $MAIN
if [ ! -e "bazel-0.10.0-dist.zip" ]
then
    wget https://github.com/bazelbuild/bazel/releases/download/0.10.0/bazel-0.10.0-dist.zip
    ok
fi
unzip -oq bazel-0.10.0-dist.zip -d bazel
ok
cd bazel
./compile.sh
ok
echo -e "\e[92madding bazel build output directory to PATH\e[0m"
export PATH=$MAIN/bazel/output:$PATH
ok

echo -e "\e[92m#### Tensorflow and Deepspeech ####\e[0m"
cd $MAIN
if [ ! -d "DeepSpeech" ]; then
    git clone https://github.com/mozilla/DeepSpeech.git
    ok
fi
cd DeepSpeech
git checkout v0.2.0
ok

cd $MAIN

if [ ! -d "tensorflow" ]; then
    git clone --recurse-submodules https://github.com/mozilla/tensorflow.git
    ok
fi
cd tensorflow
git checkout 8f1e4805401db5fcf4c0a32022a19eac7dac0dbb
ln -s ../DeepSpeech/native_client ./

#configure flags
export \
    PYTHON_BIN_PATH=${DEEPSPEECH_PYTHON_BIN_PATH} \
    PYTHON_LIB_PATH=${DEEPSPEECH_PYTHON_LIB_PATH} \
    TF_NEED_JEMALLOC=0 \
    TF_NEED_GCP=0 \
    TF_NEED_HDFS=0 \
    TF_ENABLE_XLA=0 \
    TF_NEED_GDR=0 \
    TF_NEED_VERBS=0 \
    TF_NEED_OPENCL_SYCL=0 \
    TF_NEED_CUDA=${DEEPSPEECH_CUDA_SUPPORT} \
    TF_NEED_MPI=0 \
    CC_OPT_FLAGS=${CXXFLAGS} \
    TF_SET_ANDROID_WORKSPACE=0

echo -e "\e[92mdelete chache in case this gets rebuild and let bazel clean its mess\e[0m"
rm -rf /home/${USER}/.cache
ok
bazel clean
ok

echo -e "\e[92mfix someone elses shit...\e[0m"
cp ${SCRIPT_DIR}/misc/jpeg.BUILD ${MAIN}/tensorflow/third_party/jpeg/jpeg.BUILD
ok

./configure
ok


bazel build -c opt --copt=-O3 --copt="-D_GLIBCXX_USE_CXX11_ABI=0" //native_client:libctc_decoder_with_kenlm.so
ok
bazel build --config=monolithic -c opt --copt=-O3 --copt="-D_GLIBCXX_USE_CXX11_ABI=0" --copt=-fvisibility=hidden //native_client:libdeepspeech.so //native_client:generate_trie
ok



cd $MAIN/DeepSpeech/native_client
make deepspeech
ok
mkdir -p ${SCRIPT_DIR}/DeepSpeech4Ros/libs
cp ${MAIN}/tensorflow/bazel-out/k8-py3-opt/bin/native_client/*.so ${SCRIPT_DIR}/DeepSpeech4Ros/libs/
chmod u+w ${SCRIPT_DIR}/DeepSpeech4Ros/libs/*.so
ok

duration=$((SECONDS-start))
echo -e "\e[92m#### Finished in" $duration "seconds ! You're good to go! ####\e[0m"
