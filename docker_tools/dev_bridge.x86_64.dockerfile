###############################################################################
# Modifications copyright (C) 2019 Ridecell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

ARG VERSION_X86_64="dev-x86_64-20190617_1100"
FROM apolloauto/apollo:$VERSION_X86_64
USER root

RUN  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
     apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
     apt-get update && \
     apt-get -y install \ 
       dpkg \
       ros-indigo-ros-comm \
       ros-indigo-common-msgs \
       ros-indigo-tf \
       python-catkin-tools && \
     apt-get -y clean && \
     rm -rf /home/tmp/google_styleguide && \ 
     git clone https://github.com/google/styleguide.git /home/tmp/google_styleguide && \
     cd /home/tmp/google_styleguide && \
     git reset --hard 159b4c81bbca97a9ca00f1195a37174388398a67 && \
     rm -fr .git && \
     cd -

#WORKDIR /apollo
#USER apollo
