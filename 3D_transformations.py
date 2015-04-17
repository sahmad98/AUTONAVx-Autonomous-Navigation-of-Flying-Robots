'''In this exercise we want to compute the position of the quadrotor from observations of visual markers. The 
markers are detected in images recorded from the downfacing camera of the quadrotor. We also assume the position 
of the markers in the world are known.

The following image illustrates the relations between the the world, quadrotor and marker coordinate frames. 
The transformations from the world to the marker TMW and from the quadrotor to the marker TMQ are known. Your
task is to compute the transformation from the world to the quadrotor TQW given the two other transformations.

Coordinate Frames 

In the code below implement the predefined compute_drone_pose function. Its parameters are the global marker 
pose TMW and the observed marker pose TMQ. Both are instances of the Pose3D class. It has to return the quadrotor
pose TQW. To test your code click the run button. Once you think your code is correct submit your result using 
the check button below.
'''

import numpy as np

class Pose3D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation
        
    def inv(self):
        '''
        Inversion of this Pose3D object
        
        :return inverse of self
        '''
        # TODO: implement inversion
        inv_rotation = np.transpose(self.rotation)
        inv_translation = np.dot(-inv_rotation,self.translation)
        
        return Pose3D(inv_rotation, inv_translation)
    
    def __mul__(self, other):
        '''
        Multiplication of two Pose3D objects, e.g.:
            a = Pose3D(...) # = self
            b = Pose3D(...) # = other
            c = a * b       # = return value
        
        :param other: Pose3D right hand side
        :return product of self and other
        '''
        # TODO: implement multiplication
        self_rotation = np.dot(self.rotation,other.rotation)
        self_translation = np.dot(self.rotation,other.translation)+self.translation
        return Pose3D(self_rotation, self_translation)
    
    def __str__(self):
        return "rotation:\n" + str(self.rotation) + "\ntranslation:\n" + str(self.translation.transpose())

def compute_quadrotor_pose(global_marker_pose, observed_marker_pose):
    '''
    :param global_marker_pose: Pose3D 
    :param observed_marker_pose: Pose3D
    
    :return global quadrotor pose computed from global_marker_pose and observed_marker_pose
    '''
    # TODO: implement global quadrotor pose computation
    inv = observed_marker_pose.inv()
    
    global_quadrotor_pose = global_marker_pose.__mul__(inv)

    return global_quadrotor_pose

