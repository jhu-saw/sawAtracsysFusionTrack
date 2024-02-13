import ctypes
import platform
import sys
import time
import os
import math
import numpy as np
import copy

try: 
     import cisstCommonPython as cmn
     import cisstOSAbstractionPython as osa
     import cisstVectorPython as vct
     import cisstMultiTaskPython as mts
     import cisstParameterTypesPython as prm
except ImportError:
    print 'Could not find cisst'


#Note Calling .Data on cisst objects is finicky. Need to create an object
#first. calling by ref.Data() is ok. 

# helper functions
def Vec3(x,y,z):
    return np.array([x, y , z], 'd')

def Vec4(x,y,z):
    return np.array([x, y , z, 1.0], 'd')

# Homogeneous Frame

class Frame(object):
    def __init__(self):
        self.M = np.identity(4)
         #used by the tracker
        self.IsValid = False
        self.cmnPI_180 = math.pi / 180.0
        self.cmn180_PI = 180.0 / math.pi
        self.Timestamp = 0.0

    def __normalize(self,v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v/norm

    def Copy(self):
        return copy.deepcopy(self)
        
    # this returns a reference!!!
    def Translation(self):
	    return self.M[:3, 3]
    # this returns a reference!!!
    def Rotation(self):
    	return self.M[:3,:3]
    	
    # returns the underlying numpy array
    def GetRefToNumpyArray(self):
        return self.M

    # Adapted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    # Quaternions w+ix+jy+kz are represented as [w, x, y, z].
    def Quaternion(self):
        M = self.M
        trace = M[0,0] + M[1,1] + M[2,2]
        w, x, y, z = 0.0, 0.0, 0.0, 0.0
        if( trace > 0 ):
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = ( M[2,1] - M[1,2] ) * s
            y = ( M[0,2] - M[2,0] ) * s
            z = ( M[1,0] - M[0,1] ) * s
        else:
            if ( M[0,0] > M[1,1] and M[0,0] > M[2,2] ):
              s = 2.0 * math.sqrt( 1.0 + M[0,0] - M[1,1] - M[2,2])
              w = (M[2,1] - M[1,2] ) / s
              x = 0.25 * s
              y = (M[0,1] + M[1,0] ) / s
              z = (M[0,2] + M[2,0] ) / s
            elif (M[1,1] > M[2,2]):
              s = 2.0 *  math.sqrt( 1.0 + M[1,1] - M[0,0] - M[2,2])
              w = (M[0,2] - M[2,0] ) / s
              x = (M[0,1] + M[1,0] ) / s
              y = 0.25 * s
              z = (M[1,2] + M[2,1] ) / s
            else:
              s = 2.0 *  math.sqrt( 1.0 + M[2,2] - M[0,0] - M[1,1] )
              w = (M[1,0] - M[0,1] ) / s
              x = (M[0,2] + M[2,0] ) / s
              y = (M[1,2] + M[2,1] ) / s
              z = 0.25 * s
        q = np.array([w, x ,y ,z])
        return q


    #http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    #q = wxyz
    def SetQuaternionWIJK(self, q):
        sqw = q[0]*q[0]
        sqx = q[1]*q[1]
        sqy = q[2]*q[2]
        sqz = q[3]*q[3]
        # invs (inverse square length) is only required if quaternion is not already normalised
        invs = 1.0 / (sqx + sqy + sqz + sqw)
        
        self.M[0,0] = ( sqx - sqy - sqz + sqw) * invs # since sqw + sqx + sqy + sqz =1/invs*invs
        self.M[1,1] = (-sqx + sqy - sqz + sqw) * invs 
        self.M[2,2] = (-sqx - sqy + sqz + sqw) * invs
        
        tmp1 = q[1] * q[2]
        tmp2 = q[3] * q[0]
        self.M[1,0] = 2.0 * (tmp1 + tmp2) * invs
        self.M[0,1] = 2.0 * (tmp1 - tmp2) * invs
        tmp1 = q[1] * q[3]
        tmp2 = q[2] * q[0]
        self.M[2,0] = 2.0 * (tmp1 - tmp2) * invs
        self.M[0,2] = 2.0 * (tmp1 + tmp2) * invs
        tmp1 = q[2] * q[3]
        tmp2 = q[1] * q[0]
        self.M[2,1] = 2.0 * (tmp1 + tmp2) * invs
        self.M[1,2] = 2.0 * (tmp1 - tmp2) * invs


    # returns a numpy array of XYZ position and quaternion WXZY (WIJK) 
    def GetXYZWIJK(self):
        tq = np.zeros(7, 'd')
        tq[0:3] = self.Translation()
        tq[3:7] = self.Quaternion()
        return tq

    def GetTimeXYZWIJK(self):
        tq = np.zeros(8, 'd')
        tq[0] = self.Timestamp
        tq[1:4] = self.Translation()
        tq[4:8] = self.Quaternion()
        return tq

    # returns a numpy array of XYZ position and quaternion WXZY (WIJK)
    def GetXYZIJKW(self):
        tq = np.zeros(7, 'd')
        tq[0:3] = self.Translation()
        q = self.Quaternion()
        tq[3] = q[1]
        tq[4] = q[2]
        tq[5] = q[3]
        tq[6] = q[0]
        return tq

    # double array3 for axis and angle in radians
    def SetRotAxisAngle(self, axis, angle):
        sa = math.sin(angle)
        ca = math.cos(angle)
        
        unit_axis = axis[:3] / np.linalg.norm(axis[:3])
        R = np.diag([ca, ca, ca])
        R += np.outer(unit_axis, unit_axis) * (1.0 - ca)
        unit_axis = unit_axis * sa
        R += np.array(  [[ 0.0,         -unit_axis[2],   unit_axis[1]],
                        [ unit_axis[2], 0.0,            -unit_axis[0]],
                        [-unit_axis[1], unit_axis[0],   0.0]])
        self.Rotation()[:] = R
    # Calculates inverse using simple transpose
    def Inverse(self):
        f = self.Copy()
        f.M[:3,:3] = self.M[:3,:3].T                #transpose
        f.M[:3,3]  = - np.dot(f.Rotation(), self.M[:3,3])
        return f

    # convert from cisst VctFrm3 type
    def FromVctFrm3(self, frm3):
            self.Rotation()[:] = np.copy(frm3.Rotation())
            self.Translation()[:] = np.copy(frm3.Translation())
        # convert back to cisst VctFrm3 type
    
    # convert from cisst mtsFrame4x4 type
    def FromNumpy4x4(self, np4x4):
        self.M = np.copy(np4x4)

  # convert from cisst VctFrm3 type
    def FromXYZWIJK(self, XYZWIJK):
        self.SetTranslation(XYZWIJK[0:3])
        self.SetQuaternionWIJK(XYZWIJK[3:7])

    def FromHeadTail(self, ht):
    # headtail
        self.SetTranslation(ht[0:3])
        self.RotFromZAxis(ht[3:6]-ht[0:3])
    
    def GetHeadTail(self):
        tailOffsetZ = np.array([0.0, 0.0, 100.0])
        ht =  np.concatenate((self.Translation(), self * tailOffsetZ), axis = 0)
        return ht

    def GetXYZXYZ(self):
        return self.GetHeadTail()

    def ToVctFrm3(self):
	    frm3 = vct.vctFrm3()
	    frm3.Translation()[0] = self.M[0, 3]
	    frm3.Translation()[1] = self.M[1, 3]
	    frm3.Translation()[2] = self.M[2, 3]
	    for i in range(0, 3):
	        for j in range(0, 3):
	            frm3.Rotation()[i][j] = self.M[i,j]
	    return frm3

# this does just sets the rotation from the Z axis.
    def RotFromZAxis(self, z):
        #z normalize
        zn = self.__normalize(z)
        x = [1.0,0.0,0.0]
        # check that they are no colinear
        # \todo - check for small difference rather than zero difference. 
        if (zn[0] == x[0]):
           # print "Need to choose another x vector"
            x = self.__normalize([ .3, .4,0.0])
        yn = self.__normalize(np.cross(zn, x))
        xn = np.cross(yn, zn) # orthogonal unit vectors so xn is also unit
        self.M[0:3, 0] = xn
        self.M[0:3, 1] = yn
        self.M[0:3, 2] = zn

    def ToMtsVctFrm3(self):
	    frm3 = mts.mtsVctFrm3()
	    frm3.Translation()[0] = self.M[0, 3]
	    frm3.Translation()[1] = self.M[1, 3]
	    frm3.Translation()[2] = self.M[2, 3]
	    for i in range(0, 3):
	        for j in range(0, 3):
	            frm3.Rotation()[i][j] = self.M[i,j]
	    return frm3

# Note calling .Data() on the frame without creating an object is not allowed.
# u = Frame()
#print u.ToMtsFrm4x4().Data() 
#instead create an intermediate object
#uFrm = u.ToMtsFrm4x4()
#print uFrm.Data()   - this is ok.    
    def ToMtsFrm4x4(self):
        frm = mts.mtsDoubleFrm4x4()
#        frm.Data()[0][0]  #needed to instantiate the reference.
        for i in range(0, 4):
            for j in range(0, 4):
                frm.Data()[i][j] = self.M[i,j]
        return frm
        
    # overloaded * operator , allowed inputs: Frame, array(3), array(4)
    def __mul__(self, x):
        if isinstance(x, Frame):
            f = Frame()
            f.M = np.dot(self.M, x.GetRefToNumpyArray())
            return f
        elif np.shape(x) == (4,4) or np.shape(x) == (4,):
            return np.dot(self.M,x)
        elif np.shape(x) == (3,):
	        t  = np.array([x[0],x[1],x[2],1])
	        r = np.dot(self.M,t)
	        return r[:3]
        else:
            return False
    #print itself
    def __str__(self):
        return self.M.__str__()
    #nice way to print itself
    def __repr__(self):
        return "Frame: \n" + self.M.__str__()
        
    #in Radians
    def GetRotationMagnitude(self):
        axis, angle = self.GetAxisAngle()
        return math.fabs(angle)

    def SetTranslation(self, t):
        self.M[0:3, 3] = t
    #returns axis, angle 
    def GetRotAxisAngle(self):
        # adopted from
        #http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
        #need to rewrite our own.
       
        R33 = np.copy(self.Rotation())
        # direction: unit eigenvector of R33 corresponding to eigenvalue of 1
        w, W = np.linalg.eig(R33.T)
        i = np.where(abs(np.real(w) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        direction = np.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        w, Q = np.linalg.eig(self.M)
        i = np.where(abs(np.real(w) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on direction
        cosa = (np.trace(R33) - 1.0) / 2.0
        if abs(direction[2]) > 1e-8:
            sina = (self.M[1, 0] + (cosa-1.0)*direction[0]*direction[1]) / direction[2]
        elif abs(direction[1]) > 1e-8:
            sina = (self.M[0, 2] + (cosa-1.0)*direction[0]*direction[2]) / direction[1]
        else:
            sina = (self.M[2, 1] + (cosa-1.0)*direction[1]*direction[2]) / direction[0]
        angle = math.atan2(sina, cosa)
        return direction, angle
    # simple test
    def Test(self):
        f = Frame()
        print "f:" , f
        t = f.Translation()
        t[0] = 24.0
        t[1] = -10.0
        t[2] = 2.9
        print "f after ref to translation was assigned new values: \n" , f
        d = Frame()
        g = f * d
        t1 = np.array([20.2,1,0])
        t1Res = f * t1
        print t1
        print t1Res
        t2 = np.array([20.2,1,0,1])
        t2Res = f * t2
        print t2
        print t2Res
        fInv = f.Inverse()
        
        g = Frame()
        g.SetRotAxisAngle([1.0,0.0,0.0], 10 * self.cmnPI_180)
        axis, angle = g.GetRotAxisAngle()
        if ( angle == 10*self.cmnPI_180 ):
            print "Correct Rotation"

        print "----------"

        print 'Testing Quaternion'
        f = Frame();
        f.SetTranslation([102,4,5]);
        f.SetRotAxisAngle([1.0,0.0,0.0], 10 * self.cmnPI_180)
        print "f:"
        print f
        g = f.GetXYZWIJK()
        print "f as XYZWIJK"
        print g
        print 'Replacing the quaternion - should have the same result as before: '
        f.SetQuaternionWIJK([g[3], g[4], g[5], g[6]])
        g2 = f.GetXYZWIJK()

        print 'difference  (should be zero) : ', np.sum(g - g2)
        print g - g2

        print "----------"

        z = f.GetRefToNumpyArray()[0:3,2]
        print f
        f.RotFromZAxis(z)
        print f
        print "---------- head tail ---------" 
        print f.GetHeadTail()


# f = Frame()
# f.Test()