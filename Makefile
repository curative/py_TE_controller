
all: test

test: flake8 mypy

clean:

flake8:
	flake8

mypy:
	./run_mypy.sh


.PHONY: all test clean flake8 mypy
