# Matlab interface 
Provides a function 'conex.m' for solving problems in SeDuMi format. Can be used as a replacement
for 'sedumi.m.'

Examples:
```
[x, y] = conex(A, b, c, K)
```
where A, b, c, K are as documented here 
```
http://sedumi.ie.lehigh.edu/sedumi/files/sedumi-downloads/SeDuMi_Guide_11.pdf
```

To install, add libconex.so to the path. Then run 
```
conex_setup.use_blas = 0 or 1
setup(conex_setup)
```
Note that the blas setting must match the setting used to build libconex.so.
