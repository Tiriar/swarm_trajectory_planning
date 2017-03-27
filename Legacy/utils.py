import math
import sys

__author__ = 'BRICH'


def norm(v):
    """
    Computes vector norm
    :param v: input vector
    :return: norm of vector v
    """
    tmp = 0
    for i in v:
        tmp += i**2
    return tmp**0.5


def add(v1, v2):
    """
    Vector addition
    :param v1: first vector
    :param v2: second vector
    :return: v1+v2
    """
    out = [0]*len(v1)
    for i in range(len(out)):
        out[i] = v1[i]+v2[i]
    return out


def sub(v1, v2):
    """
    Vector substitution
    :param v1: first vector
    :param v2: second vector
    :return: v1-v2
    """
    out = [0]*len(v1)
    for i in range(len(out)):
        out[i] = v1[i]-v2[i]
    return out


def mul(v, m):
    """
    Vector multiplication
    :param v: input vector
    :param m: multiplication constant
    :return: v*m
    """
    out = [0]*len(v)
    for i in range(len(out)):
        out[i] = v[i]*m
    return out


def dot(v1, v2):
    """
    Vector dot product
    :param v1: first vector
    :param v2: second vector
    :return: dot product
    """
    tmp = 0
    for i in range(len(v1)):
        tmp += v1[i]*v2[i]
    return tmp


def angle(v1, v2):
    """
    Angle between two vectors
    :param v1: first vector
    :param v2: second vector
    :return: angle
    """
    if norm(v1) == 0 or norm(v2) == 0:
        return math.pi
    return math.acos(dot(v1, v2)/(norm(v1)*norm(v2)))


def closest(point, points, number):
    """
    Finds closest points from given list to given points
    :param point: coords of given point
    :param points: list of coords of points
    :param number: number of closest to be found
    :return: indices of closest points
    """
    cl = [sys.maxsize]*number
    clidx = [sys.maxsize]*number
    temp = point
    for i in range(len(points)):
        nrm = norm(sub(temp, points[i]))
        if nrm < max(cl) and nrm != 0:
            idx = cl.index(max(cl))
            cl[idx] = nrm
            clidx[idx] = i
    return clidx


def point_line(p, l):
    """
    Computes distance of point A from line p
    :param p: p[p0, p1]
    :param l: l[a, b, c]; l: ax+by+c=0
    :return: distance
    """
    return abs(l[0]*p[0]+l[1]*p[1]+l[2])/math.sqrt(l[0]**2+l[1]**2)


def get_color(pos):
    """
    Computes RGB color for VRep points visualisation
    :param pos: [0-1] color scale
    :return: RGB color between green and red based on pos
    """
    out = [0, 0, 0]
    if pos < 0.5:
        out[0] = 255*pos*2
        out[1] = 255
    else:
        out[0] = 255
        out[1] = 255-(pos-0.5)*2*255
    out[0] = math.floor(out[0])
    out[1] = math.floor(out[1])
    return out


def line(p1, p2):
    """
    Computes line coefficients
    :param p1: first line point [x,y]
    :param p2: second line point [x,y]
    :return: a, b, c coefficients (line: ax+by+c=0)
    """
    a = p1[1]-p2[1]
    b = p2[0]-p1[0]
    c = p1[0]*p2[1]-p2[0]*p1[1]
    return [a, b, -c]


def intersection(a1, a2, b1, b2):
    """
    Computes line segments intersection
    :param a1: start point of first segment [x,y]
    :param a2: end point of first segment [x,y]
    :param b1: start point of second segment [x,y]
    :param b2: end point of second segment [x,y]
    :return: found intersection or None
    """
    center1 = mul(add(a1, a2), 0.5)
    center2 = mul(add(b1, b2), 0.5)
    len1 = norm(sub(a2, a1))
    len2 = norm(sub(b2, b1))

    l1 = line(a1, a2)
    l2 = line(b1, b2)

    d = l1[0]*l2[1]-l1[1]*l2[0]
    dx = l1[2]*l2[1]-l1[1]*l2[2]
    dy = l1[0]*l2[2]-l1[2]*l2[0]
    if d != 0:
        output = [float(dx)/d, float(dy)/d]
        if norm(sub(center1, output)) > len1/2 or norm(sub(center2, output)) > len2/2:
            return None
        return output
    return None


def outside_point(obj, inter, p1, p2):
    """
    Decides which endpoint leads outside of the given object
    :param obj: rectangular object edges
    :param inter: starting point
    :param p1: endpoint #1
    :param p2: endpoint #2
    :return: outside point
    """
    c1 = mul(add(obj[0], obj[1]), 0.5)
    c2 = mul(add(obj[2], obj[3]), 0.5)
    center = mul(add(c1, c2), 0.5)

    vec1 = sub(p1, inter)
    vec1 = mul(vec1, 1/norm(vec1))
    vec2 = sub(p2, inter)
    vec2 = mul(vec2, 1/norm(vec2))

    a = add(inter, vec1)
    b = add(inter, vec2)

    if norm(sub(a, center)) < norm(sub(b, center)):
        return p2
    return p1


def merge_obstacles(obj, obstacles, ignore):
    """
    Merges rectangular obstacles with given object if they intercept
    :param obj: input object index in obstacles
    :param obstacles: set of obstacles given by corner coordinates
    :param ignore: indexes of obstacles to be ignored
    :return: merged object
    """
    visited = []
    merged = []
    a1 = obstacles[obj][0]
    a2 = obstacles[obj][1]
    objidx = obj
    pntidx = 0
    currobj = obj
    lastobj = obj

    while True:
        merged.append(a1)
        if objidx not in visited:
            visited.append(objidx)
        overlap = False
        mindist = sys.maxsize
        closest = [[0, 0], [0, 0], [0, 0]]  # [intersection, b1, b2]
        for i in range(len(obstacles)):
            if i != currobj and i != lastobj and i not in ignore:
                for j in range(4):
                    b1 = obstacles[i][j]
                    if j == 3:
                        b2 = obstacles[i][0]
                    else:
                        b2 = obstacles[i][j+1]
                    inter = intersection(a1, a2, b1, b2)
                    if inter is not None:
                        overlap = True
                        dist = norm(sub(a1, inter))
                        if dist < mindist:
                            mindist = dist
                            closest[0] = inter
                            closest[1] = b1
                            closest[2] = b2
                            objidx = i
                            pntidx = j
        if overlap:
            a1 = closest[0]
            a2 = outside_point(obstacles[currobj], closest[0], closest[1], closest[2])
            lastobj = currobj
            currobj = objidx
        else:
            a1 = a2
            pntidx = (pntidx+1) % 4
            if pntidx == 3:
                a2 = obstacles[objidx][0]
            else:
                a2 = obstacles[objidx][pntidx+1]
            lastobj = currobj
        if a1 == merged[0]:
            break
    return merged, visited
