FROM archlinux:base-20220515.0.56491

RUN pacman --noconfirm -Syu

# Install basic programs and custom glibc
RUN pacman --noconfirm -S \
    git \
    wget \
    make \
    cmake \
    unzip \
    arm-none-eabi-gcc \
    arm-none-eabi-newlib
    
# Install clangd
ARG CLANGD="https://github.com/clangd/clangd/releases/download/14.0.3/clangd-linux-14.0.3.zip"

RUN wget ${CLANGD} -O ~/temp.zip
RUN unzip ~/temp.zip -d ~/clangd
RUN cp ~/clangd/*/bin/clangd /usr/local/bin/
RUN rm ~/temp.zip && rm -R ~/clangd