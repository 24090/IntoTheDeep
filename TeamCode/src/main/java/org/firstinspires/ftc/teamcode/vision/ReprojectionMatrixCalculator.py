import numpy as np, scipy
res_scale = 360/480

focal_length       = (640, 640) #(822.317, 822.317)
principal_point    = (320, 240) #(319.495, 242.502)
rotation_xyz_euler = [50, 0, 0]
degrees            = True
position           = [0, 0, 12.15732]

# find intrinsics matrix
intrinsics_matrix = np.asarray([
						[focal_length[0], 0,               principal_point[0]],
						[0, 	          focal_length[1], principal_point[1]],
						[0,               0,               1/res_scale       ]
])*res_scale

# find extrinsics matrix
rotation_matrix = scipy.spatial.transform.Rotation.from_euler("XYZ", rotation_xyz_euler, degrees).as_matrix()
translation_vector = np.asarray(position).reshape((3,1))
homogenous_transformation_matrix = np.linalg.inv(
	np.vstack((
		np.hstack((rotation_matrix, translation_vector)),
		          [0,     0,     0,          1        ]
	))
)
extrinsics_matrix = homogenous_transformation_matrix[:-1]

# find reprojection matrix

projection_matrix = (intrinsics_matrix @ extrinsics_matrix)
reprojection_matrix = np.linalg.inv(projection_matrix[:, [0,1,3]])
# generate code to make it
print("\n// generated with ReprojectionMatrixCalculator.py")
for row_num in range(3):
	print(f"m1.put({row_num}, 0, {reprojection_matrix[row_num][0]: e}); m1.put({row_num}, 1, {reprojection_matrix[row_num][1]: e}); m1.put({row_num}, 2, {reprojection_matrix[row_num][2]: e});")
print("")

# test functions
def project(x, y, z):
	v = projection_matrix @ [x, y, z, 1]
	v = v/v[2]
	return v

def reproject(u, v):
	v = reprojection_matrix @ [u, v, 1]
	v = v/v[2]
	return v
