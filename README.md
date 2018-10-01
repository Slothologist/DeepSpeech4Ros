# DeepSpeech4Ros
Mozilla Deepspeech for Ros via Jackaudio as part of a larger Pipeline

## Requirements

- Jack audio

- ROS

- Deepspeech (install script included)

- Boost library

- soxr library

Deepspeech is a bit tricky to install, since we need a linkable library and not just a python wheel.
Therefore we must build and install Deepspeech ourselves, along with its dependencies, tensorflow and bazel.

But worry not friend, a install script for ubuntu is included in this repo (install_deepspeech.sh) to take all the effort out of that.
It will create shared libraries for deepspeech and copy them to the DeepSpeech4Ros/libs folder, where the CMakeLists.txt will be looking for it.

Also included is a script to install the necessary system dependencies for tensorflow and bazel (install_prerequisites.sh).

You will need to install Jack, Ros, Boost and soxr yourself.