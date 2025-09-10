.PHONY: configure-debug configure-release build-debug build test clean format

configure-debug:
	mkdir -p build
	cd build && cmake .. -GNinja -DCXX=clang++ -DCC=clang -DCMAKE_BUILD_TYPE=Debug -DDIAMOND_FEM_BUILD_TESTS=ON

configure-release:
	mkdir -p build
	cd build && cmake .. -GNinja -DCXX=clang++ -DCC=clang -DCMAKE_BUILD_TYPE=Release

build-debug: configure-debug
	cd build && cmake --build .

build: configure-release
	cd build && cmake --build .

test: build-debug
	cd build && ctest --output-on-failure

clean:
	rm -rf build

format:
	clang-format -style=file -i include/**/*.hpp
