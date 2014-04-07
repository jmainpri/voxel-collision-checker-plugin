# Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
#   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-
'''Functions for defining, extracting data from, and serializing transform matrices'''
from numpy import *

def MakeTransform(rot,trans):
    # print trans
    # print type(trans)
    # print size(trans,0)
    # print size(trans,1)
    # print rot
    # print type(rot)
    # print size(rot,0)
    # print size(rot,1)
    if size(rot,0) == 9 and size(rot,1) == 1:
        tm = rot.reshape(3,3)
    elif size(rot,0) == 3 and size(rot,1) == 3:
        tm = rot
    else:
        print('rotation improperly specified');

    if size(trans,0) == 3 and size(trans,1) == 1:
        #tm = bmat('tm trans')
        tm = bmat([tm,trans])
    elif size(trans,0) == 1 and size(trans,1) == 3:
        # Note: for some reason the following line does not work anymore.
        #       There's even an open ticket about it being a bug:
        #       http://projects.scipy.org/numpy/ticket/680
        #       
        #       Apparently it won't be fixed. So we change the function call
        # tm = bmat('tm trans.T')
        tm = bmat([tm,trans.T])
    else:
        print('translation improperly specified');
    
    lastrow = mat([0,0,0,1])
    return bmat('tm; lastrow')

'''Rodrigues formula
Input: 1x3 array of rotations about x, y, and z
Output: 3x3 rotation matrix'''
def rodrigues(r):
    def S(n):
        Sn = array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
        return Sn
    theta = linalg.norm(r)
    if theta > 1e-30:
        n = r/theta
        Sn = S(n)
        R = eye(3) + sin(theta)*Sn + (1-cos(theta))*dot(Sn,Sn)
    else:
        Sr = S(r)
        theta2 = theta**2
        R = eye(3) + (1-theta2/6.)*Sr + (.5-theta2/24.)*dot(Sr,Sr)
    return mat(R)
