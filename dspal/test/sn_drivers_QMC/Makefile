all: 
	mkdir -p out && cd out && cmake -Wno-dev ../ \
		-DCMAKE_TOOLCHAIN_FILE=external/cmake_hexagon/toolchain/Toolchain-qurt.cmake \
		-DQC_SOC_TARGET=APQ8074
	cd out && make -j 4 && rm libsnav_q6.so && ${HEXAGON_TOOLS_ROOT}/bin/hexagon-strip *.so
	
clean:
	rm -rf out *~
