from numpy cimport ndarray as ar
cimport numpy as np
import numpy as np
cimport cython

@cython.boundscheck(False)
@cython.wraparound(False)
def tonumpyarray(xy, height, width):
    cdef int i, j, h=height, w=width
    cdef ar[np.uint8_t, ndim=2] new = np.empty((h, w), dtype=np.uint8)
    for i in xrange(h):
        for j in xrange(w):
            new[i,j] = xy[i * w + j]
    return new
