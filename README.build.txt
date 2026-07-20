sudo apt update
sudo apt install -y git ninja-build meson libglib2.0-dev libpixman-1-dev libgcrypt20-dev
sudo apt build-dep -y qemu

mkdir build
cd build
../configure --target-list=riscv32-softmmu \
            --enable-gcrypt \
            --enable-slirp \
            --enable-sdl \
            --disable-strip \
            --disable-user \
            --disable-capstone \
            --disable-vnc \
            --disable-gtk
ninja -C .
