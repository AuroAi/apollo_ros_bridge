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
