import numpy as np

DTYPE = np.float32

class Quaternion:
    def __init__(self, w:float=1, x:float=0, y:float=0, z:float=0):
        self.w = DTYPE(w)
        self.x = DTYPE(x)
        self.y = DTYPE(y)
        self.z = DTYPE(z)
        self._norm = np.sqrt(self.w*self.w+self.x*self.x+self.y*self.y+self.z*self.z, dtype=DTYPE)

    def toNumpy(self):
        """convert to an (4,1) numpy array"""
        return np.array([self.w,self.x,self.y,self.z], dtype=DTYPE).reshape((4,1))
    
    def unit(self):
        """return the unit quaternion"""
        return Quaternion(self.w/self._norm,self.x/self._norm,self.y/self._norm,self.z/self._norm)

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def reverse(self):
        """return the reverse rotation representation as the same as the transpose op of rotation matrix"""
        return Quaternion(-self.w,self.x,self.y,self.z)

    def inverse(self):
        return Quaternion(self.w/(self._norm*self._norm),-self.x/(self._norm*self._norm),-self.y/(self._norm*self._norm),-self.z/(self._norm*self._norm))
    
    def __str__(self) -> str:
        return '['+str(self.w)+', '+str(self.x)+', '+str(self.y)+', '+str(self.z)+']'


def quat_to_rpy(q:Quaternion) -> np.ndarray:
    """
    Convert a quaternion to RPY. Return
    angles in (roll, pitch, yaw).
    """
    rpy = np.zeros((3,1), dtype=DTYPE)
    as_ = np.min([-2.*(q.x*q.z-q.w*q.y),.99999])
    # roll
    rpy[0] = np.arctan2(2.*(q.y*q.z+q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    # pitch
    rpy[1] = np.arcsin(as_)
    # yaw
    rpy[2] = np.arctan2(2.*(q.x*q.y+q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    return rpy
