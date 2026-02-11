all: check

check: ../libconex.so conex.py
	python3 -m py_compile conex.py ConexProgram.py

.PHONY: clean

clean:
	rm -f *.so
	rm -f *.o
	rm -f *.out
	rm -f *.pyc
