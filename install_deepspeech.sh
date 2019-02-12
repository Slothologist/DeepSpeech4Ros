#!/usr/bin/env bash


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
DIR=$(pwd)

# config values
DEEPSPEECH_CUDA_SUPPORT=${1:-"0"} 
DEEPSPEECH_INSTALL_PREFIX=${2:-$DIR}
DEEPSPEECH_CXX_OPTS=${3:-"-march=native"} 

echo "Building deepspeech with cuda support? $DEEPSPEECH_CUDA_SUPPORT"
echo "Building deepspeech with install prefix? $DEEPSPEECH_INSTALL_PREFIX"
echo "Building deepspeech with c++ opts? $DEEPSPEECH_CXX_OPTS"

function ok (){
  if [ $? -eq 0 ]; then
     echo -e "\e[92m-----> OK, going on!\e[0m"
  else
     echo -e "\e[31m-----> Failure, exiting install script, noooo :|\e[0m"
     exit $?
  fi
}

export MAIN=$DIR/deepspeech_install
mkdir -p $MAIN
CPU_CORES='nproc --all'
start=$SECONDS

echo -e "\e[92m#### Bazel ####\e[0m"
cd $MAIN
if [ ! -e "bazel-0.15.0-dist.zip" ]
then
    wget https://github.com/bazelbuild/bazel/releases/download/0.15.0/bazel-0.15.0-dist.zip
    ok
fi
unzip -oq bazel-0.15.0-dist.zip -d bazel
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
git checkout v0.4.1
ok

cd $MAIN

if [ ! -d "tensorflow" ]; then
    git clone --recurse-submodules https://github.com/mozilla/tensorflow.git
    ok
fi
cd tensorflow
git checkout r1.12
ln -s ../DeepSpeech/native_client ./


#configure flags
export \
    PYTHON_BIN_PATH=/usr/bin/python3 \
    PYTHON_LIB_PATH=/usr/lib/python3/dist-packages \
    TF_NEED_JEMALLOC=0 \
    TF_NEED_GCP=0 \
    TF_NEED_HDFS=0 \
    TF_ENABLE_XLA=0 \
    TF_NEED_GDR=0 \
    TF_NEED_VERBS=0 \
    TF_NEED_OPENCL_SYCL=0 \
    TF_NEED_CUDA=$DEEPSPEECH_CUDA_SUPPORT \
    TF_NEED_MPI=0 \
    TF_NEED_ROCM=0 \
    TF_NEED_IGNITE=0 \
    TF_DOWNLOAD_CLANG=0 \
    CC_OPT_FLAGS=$DEEPSPEECH_CXX_OPTS \
    TF_SET_ANDROID_WORKSPACE=0

echo -e "\e[92mdelete chache in case this gets rebuild and let bazel clean its mess\e[0m"
rm -rf /home/${USER}/.cache
ok
bazel clean
ok


./configure
ok

bazel build --config=monolithic -c opt --copt=-O3 --copt="-D_GLIBCXX_USE_CXX11_ABI=0" --copt=-fvisibility=hidden //native_client:libdeepspeech.so //native_client:generate_trie
ok



cd $MAIN/DeepSpeech/native_client
make deepspeech
ok
mkdir -p $DEEPSPEECH_INSTALL_PREFIX/lib/deepspeech
cp ${MAIN}/tensorflow/bazel-out/k8*/bin/native_client/*.so $DEEPSPEECH_INSTALL_PREFIX/lib/deepspeech/
ok
chmod +w $DEEPSPEECH_INSTALL_PREFIX/lib/deepspeech/libdeepspeech.so
ok
mkdir -p $DEEPSPEECH_INSTALL_PREFIX/include/deepspeech
cp ${MAIN}/DeepSpeech/native_client/deepspeech.h $DEEPSPEECH_INSTALL_PREFIX/include/deepspeech/
ok
chmod +w $DEEPSPEECH_INSTALL_PREFIX/include/deepspeech/deepspeech.h
ok

duration=$((SECONDS-start))
echo -e "\e[92m#### Finished in" $duration "seconds ! You're good to go! ####\e[0m"
