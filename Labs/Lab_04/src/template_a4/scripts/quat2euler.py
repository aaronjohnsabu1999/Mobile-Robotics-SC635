from tf.transformations import euler_from_quaternion, quaternion_from_euler

def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)

def euler2quat(r, p, y):
    return quaternion_from_euler(r,p,y)
