all: _conex

.PHONY: clean

_conex:
	@command -v cmake >/dev/null || { echo "cmake is required to build _conex"; exit 1; }
	cmake -S . -B build -DPython3_EXECUTABLE="$$(command -v python3)"
	cmake --build build -j

clean:
	rm -f *.so
	rm -f *.o
	rm -f *.out
	rm -f *.pyc
	rm -rf build
