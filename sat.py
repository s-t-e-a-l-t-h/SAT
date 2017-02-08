# implementation of SAP algorithm in 2d
# detection of collision of two faces
# if faces are touched, return False too

import Plot as Plt
import numpy as np

# constants

# round to
PRECISION = 10


def projection(v, t):
    t, v = np.array(t), np.array(v)
    return (np.dot(v, t) / np.dot(t, t)) * t


def to_tangential(tn, nn, pt):
    m = [[np.dot(np.array([1.0, 0.0]), tn), np.dot(np.array([0.0, 1.0]), tn)],
         [np.dot(np.array([1.0, 0.0]), nn), np.dot(np.array([0.0, 1.0]), nn)]]
    return np.dot(m, pt)


def separating_axis_theorem(tri1, tri2):
    # each edge of face tri1 and tri2
    # number of face vertices
    nfv = len(tri1)
    edges = np.array(
        [[tri1[i], tri1[i + 1]] for i in range(-1, nfv - 1)] + [[tri2[i], tri2[i + 1]] for i in range(-1, nfv - 1)])

    for edge in edges:
        # tangent vector of testing line
        tangent = (edge[1] - edge[0]) / np.linalg.norm(edge[1] - edge[0])
        # normal vector of testing line
        normal = -tangent[1], tangent[0]

        # we flip tangent and normal vector (we will project all points of each face on the normal line of
        # current edge)
        tangent, normal = normal, tangent

        # projection of each vertex on testing line
        projection_t1 = [to_tangential(tangent, normal, projection(vertex, tangent)).tolist() for vertex in tri1]
        projection_t2 = [to_tangential(tangent, normal, projection(vertex, tangent)).tolist() for vertex in tri2]

        # new x coo (y should be zero now)
        projection_t1_newx, projection_t2_newx = list(zip(*projection_t1))[0], list(zip(*projection_t2))[0]

        # maximal length projected of each face
        projection_edge1, projection_edge2 = [round(min(projection_t1_newx), PRECISION),
                                              round(max(projection_t1_newx), PRECISION)], \
                                             [round(min(projection_t2_newx), PRECISION),
                                              round(max(projection_t2_newx), PRECISION)]

        projection_edge = [projection_edge1, projection_edge2]
        projection_edge.sort(key=lambda x: x[0])

        # if intervals connected in point or separated return True
        if projection_edge[0][1] <= projection_edge[1][0]:
            return True

    return False


def intersection(t1, t2):
    return not separating_axis_theorem(t1, t2)


# not intersection example:
# faces = [[[0.0, 0.5],
#           [1.0, 0.5],
#           [0.0, 1.0]],
#          [[0.0, 0.0],
#           [1.0, 0.4],
#           [0.0, -.7]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)

# not intersection example:
# faces = [[[0.0, 0.5],
#           [1.0, 0.5],
#           [0.0, 1.0]],
#          [[0.0, 0.0],
#           [1.0, 0.3],
#           [0.0, 0.3]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)

# # overllaping example:
# faces = [[[0.0, 0.5],
#           [1.0, 0.5],
#           [0.0, 1.0]],
#          [[0.0, 0.0],
#           [1.0, 0.5],
#           [0.0, 0.5]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)

# # overllaping example:
# faces = [[[0.0, 0.0],
#           [1.0, 0.0],
#           [0.0, 1.0]],
#          [[-1., 1.0],
#           [0.0, 1.0],
#           [0.0, 0.0]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)

# # touching example
# faces = [[[0.0, 0.0],
#           [1.0, 0.0],
#           [0.0, 1.0]],
#          [[0.0, 0.0],
#           [-.1, -.5],
#           [-.5, -.5]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)

# # intersection example
# faces = [[[0.0, 0.5],
#           [1.0, 0.5],
#           [0.0, 1.0]],
#          [[0.0, 2.0],
#           [1.0, 0.5],
#           [0.0, 0.5]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)

#
# # intersection example
# faces = [[[0.0, 0.0],
#           [2.0, 0.0],
#           [0.0, 2.0]],
#          [[-.9, -.9],
#           [-3., 3.5],
#           [9.5, 1.0]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)


# faces = [[[-0.4760028, 0.37223806],
#           [-0.33617698, 0.04619783],
#           [-0.19737227, 0.31664461]],
#          [[0., 0.],
#           [-0.33617698, 0.04619783],
#           [-0.19737227, 0.31664461]]]
#
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 3):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)


# # 4 dim faces:
# faces = [[[0.0, 0.5],
#           [1.0, 0.5],
#           [1.0, 1.0],
#           [0.0, 1.0]],
#          [[0.5, 0.5],
#           [4.0, 0.5],
#           [4.0, 3.0],
#           [0.5, 3.0]]]
# print(intersection(faces[0], faces[1]))
# for i in range(0, 2):
#     for j in range(0, 4):
#         faces[i][j].append(0.0)
# Plt.plot_3d(faces=[faces], faces_view=True, normals_view=False, points_view=False, face_color=[["r", "g"]],
#             face_alpha=0.5, azim=-90, elev=90)


