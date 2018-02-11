#!/usr/bin/env python

import numpy as np
import time

# Copied from https://github.com/wiseodd/hipsternet
# Shouldn't need to directly use this
def _get_im2col_indices(x_shape, field_height, field_width, padding=0, stride=1):
  # First figure out what the size of the output should be
  N, C, H, W = x_shape
  assert (H + 2 * padding - field_height) % stride == 0
  assert (W + 2 * padding - field_height) % stride == 0
  out_height = (H + 2 * padding - field_height) / stride + 1
  out_width = (W + 2 * padding - field_width) / stride + 1

  i0 = np.repeat(np.arange(field_height), field_width)
  i0 = np.tile(i0, C)
  i1 = stride * np.repeat(np.arange(out_height), out_width)
  j0 = np.tile(np.arange(field_width), field_height * C)
  j1 = stride * np.tile(np.arange(out_width), out_height)
  i = i0.reshape(-1, 1) + i1.reshape(1, -1)
  j = j0.reshape(-1, 1) + j1.reshape(1, -1)

  k = np.repeat(np.arange(C), field_height * field_width).reshape(-1, 1)

  return (k, i, j)

# Copied from https://github.com/wiseodd/hipsternet
# Shouldn't need to directly use this
def _im2col_indices(x, field_height, field_width, padding=0, stride=1):
  """ An implementation of im2col based on some fancy indexing """
  # Zero-pad the input
  p = padding
  x_padded = np.pad(x, ((0, 0), (0, 0), (p, p), (p, p)), mode='constant')

  k, i, j = _get_im2col_indices(x.shape, field_height, field_width, padding,
                               stride)

  cols = x_padded[:, k, i, j]
  C = x.shape[1]
  cols = cols.transpose(1, 2, 0).reshape(field_height * field_width * C, -1)
  return cols

# Adapted from https://github.com/wiseodd/hipsternet
# X is an image with dimensions (X_height, X_width)
# W is a filter with dimensions (W_channels, W_height, W_width)
# stride is the amount of pixels the filter is slid
# padding is the number of border pixels that are added to the image
# Returns the result of the convolution with dimensions (out_channels, out_height, out_width)
def convolve(X, W, stride=1, padding=0):
    X = X[np.newaxis, np.newaxis, :, :]
    W = W[:,np.newaxis,:,:]
    n_filters, d_filter, h_filter, w_filter = W.shape
    n_x, d_x, h_x, w_x = X.shape
    h_out = (h_x - h_filter + 2 * padding) / stride + 1
    w_out = (w_x - w_filter + 2 * padding) / stride + 1

    #if not h_out.is_integer() or not w_out.is_integer():
    #    raise Exception('Invalid output dimension!')

    h_out, w_out = int(h_out), int(w_out)

    X_col = _im2col_indices(X, h_filter, w_filter, padding=padding, stride=stride)
    W_col = W.reshape(n_filters, -1)

    #print W_col.shape
    #print X_col.shape
    #out = W_col @ X_col 
    out = np.dot(W_col, X_col)
    out = out.reshape(n_filters, h_out, w_out, n_x)
    out = out.transpose(3, 0, 1, 2)

    return out.squeeze(axis=0)
    
if __name__ == '__main__':
  #X = np.array([[10,11,12,13],[14,15,16,17],[18,19,20,21],[22,23,24,25]])
  #X = np.random.rand(80,120)
  #X = X[np.newaxis, np.newaxis, :, :]
  #W = np.array([[[1,1],[1,1]],[[2,2],[2,2]]])
  #W = np.random.rand(100,80,20)
  #W = W[:,np.newaxis,:,:]
  
  X = np.array([[10,11,12,13],[14,15,16,17],[18,19,20,21],[22,23,24,25]])
  W = np.array([[[1,2],[3,4]],[[2,2],[2,2]]])
  
  print 'X:'
  print X
  print 'W:'
  print W
  print 'Res:'
  start = time.time()
  out = convolve(X,W, padding=0)
  print out
  print time.time()-start

