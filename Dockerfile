FROM ros:melodic

# Update and grab some dependencies
RUN apt-get update
RUN apt-get install -y libeigen3-dev libgoogle-glog-dev libgtest-dev ros-melodic-tf

# Download and install GTest (needed for building and unit tests)
WORKDIR /usr/src/gtest
RUN cmake .
RUN make
RUN cp *.a /usr/lib

# Download and install Z3
WORKDIR /tmp
RUN apt-get install -y unzip wget
RUN wget https://github.com/Z3Prover/z3/releases/download/z3-4.8.9/z3-4.8.9-x64-ubuntu-16.04.zip
RUN unzip z3-4.8.9-x64-ubuntu-16.04
RUN cp z3-4.8.9-x64-ubuntu-16.04/bin/libz3.* /usr/bin
RUN cp z3-4.8.9-x64-ubuntu-16.04/include/* /usr/include

# Add code and build
RUN mkdir -p /cpp-pips
WORKDIR /cpp-pips
COPY . .
RUN ./docker_build.sh

# Extra Docker setup
COPY docker_entrypoint.sh /docker_entrypoint.sh
ENTRYPOINT [ "/docker_entrypoint.sh" ]
CMD [ "bash" ]