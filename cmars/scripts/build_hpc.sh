!#/bin/sh

module load gcc/11.2.0
module load cuda/12.0
module load eigen/3.4.0

cd .. && rm -r build && mkdir build && cd build
cmake .. -DCMAKE_C_COMPILER=$(which gcc) -DCMAKE_CXX_COMPILER=$(which g++) \
             -DChrono_DIR="$HOME/chrono-wisc/build/cmake"   \
             -DIMAGE_DATA_LIB="$HOME/drive-primer/image-data/build/libimage_data.a" \
             -DIMAGE_DATA_INCLUDES="$HOME/drive-primer/image-data"
make