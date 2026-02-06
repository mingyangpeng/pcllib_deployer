cd /root/code/test/build
# cmake .. -DCMAKE_BUILD_TYPE=Release  
time cmake --build . -j20
ccache -s